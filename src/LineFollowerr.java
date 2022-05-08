import java.io.FileWriter;
import java.io.IOException;
import java.util.Stack;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.robotics.Color;
import lejos.utility.Delay;

public class LineFollower {
	EV3LargeRegulatedMotor motorA;
	EV3LargeRegulatedMotor motorB;
	LinesData linesData;
	ColorSensor colorSensor;
	Color currentColor;

	double kp = 11;
	double ki = 0;
	double kd = 0;
	int tp = 350;

	double integral = 0;
	double error = 0;
	double lastError = 0;
	double derivative = 0;
	double turn = 0;
	double dt = 1;

	int distanceLine;
	int distanceBackground;
	int distanceMiddleLb;
	int distanceIntersection;
	int direction;

	boolean isLost = false;
	long startTime;
	long lastTime = 0;

	long duration;
	Stack<Move> moves;
	int outside = 0;

	Move lastMove;
	private boolean timing;
	private long endTime;

	public LineFollower(Port port) {
		motorA = new EV3LargeRegulatedMotor(MotorPort.A);
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);
		colorSensor = new ColorSensor(port);
		linesData = new LinesData(colorSensor);
		moves = new Stack<>();
		lastMove = new Move(0, 0);
	}

	public void start() {
		while (Button.UP.isUp()) {
			collectData();
			checkIfLost();
			pid();
		}
	}

	private void checkIfLost() {
		System.out.println(distanceBackground);

		if (distanceBackground <= 10) {
			if (!timing) {
				startTime = System.nanoTime();
				timing = true;
			}

			isLost = true;
			outside++;
			if (outside == 100) {
				Sound.beep();
				endTime = System.nanoTime();
				findLine(endTime - startTime);
			}
		} else {
			timing = false;
			outside = 0;
			isLost = false;
		}
	}

	public void collectData() {
		currentColor = colorSensor.getColor();
		distanceLine = LinesData.calculateColorDistance(currentColor, linesData.getLine());
		distanceBackground = LinesData.calculateColorDistance(currentColor, linesData.getBackground());
		distanceMiddleLb = LinesData.calculateColorDistance(currentColor, linesData.getMiddleLineBgColor());
		distanceIntersection = LinesData.calculateColorDistance(currentColor, linesData.getIntersectionColor());
		direction = getDirection();
	}

	public void pid() {
		error = distanceMiddleLb * direction;
		derivative = error - lastError;
		turn = (kp * error) + (ki * integral) + (kd * derivative);
		/* recordData(kp * error, ki * integral, kd * derivative); */
		int leftSpeed = (int) (tp + turn);
		int rightSpeed = (int) (tp - turn);
		setMotorSpeed(leftSpeed, rightSpeed);
		lastError = error;
	}

	public void findLine(long l) {
		isLost = false;
		setMotorSpeed(-lastMove.leftSpeed, -lastMove.rightSpeed);
		Delay.nsDelay(l);

		while (true) {
			setMotorSpeed(tp, tp);
			collectData();
			if (getDirection() != -1) {
				break;
			}
		}
	}

	public int getDirection() {
		int min = Math.min(Math.min(distanceIntersection, distanceLine),
				(Math.min(distanceBackground, distanceMiddleLb)));

		direction = 0;
		if (min == distanceLine) {
			direction = 1;
		} else if (min == distanceBackground || min == distanceIntersection) {
			direction = -1;
		} else if (min == distanceMiddleLb) {
			direction = 0;
		}

		return direction;
	}

	public void setMotorSpeed(int leftSpeed, int rightSpeed) {

		leftSpeed = adjustSpeed(leftSpeed);
		rightSpeed = adjustSpeed(rightSpeed);

		motorA.setSpeed(leftSpeed);
		motorB.setSpeed(rightSpeed);
		lastMove.setLeftSpeed(leftSpeed);
		lastMove.setRightSpeed(rightSpeed);

		if ((leftSpeed) < 0) {
			leftSpeed *= -1;
			motorA.setSpeed(leftSpeed);
			motorA.backward();
		} else {
			motorA.forward();
		}

		if ((rightSpeed) < 0) {
			rightSpeed *= -1;
			motorB.setSpeed(rightSpeed);
			motorB.backward();

		} else {
			motorB.forward();
		}
	}

	public int adjustSpeed(int speed) {
		return Math.max(Math.min(speed, tp), -tp);
	}

	public void recordData(double error, double integral, double derivative) {
		try {
			FileWriter csvWriter = new FileWriter("data.csv", true);
			csvWriter.append("" + error);
			csvWriter.append(",");
			csvWriter.append("" + integral);
			csvWriter.append(",");
			csvWriter.append("" + derivative);
			csvWriter.append("\n");
			csvWriter.flush();
			csvWriter.close();

		} catch (IOException e) {
			// TODO Auto-generated catch block
		}

	}
}
