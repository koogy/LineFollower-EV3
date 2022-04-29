import java.io.FileWriter;
import java.io.IOException;
import java.util.Stack;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.utility.Delay;

public class LineFollowerr {
	EV3LargeRegulatedMotor motorA;
	EV3LargeRegulatedMotor motorB;
	LinesData linesData;
	ColorSensor colorSensor;
	Color currentColor;

	double kp = 6; // TP/error
	double ki = 0;
	double kd = 0;
	int tp = 200;

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
	long endTime = 0;

	long lastTime = 0;
	long duration;
	Stack<Move> moves;
	int index = 0;

	public LineFollowerr(Port port) {
		motorA = new EV3LargeRegulatedMotor(MotorPort.A);
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);

		colorSensor = new ColorSensor(port);
		linesData = new LinesData(colorSensor);
		moves = new Stack<>();
	}

	public void start() {
		/* Button.waitForAnyPress(); */
		int outside = 0;

		while (Button.ENTER.isUp()) {
			collectData();

			if (distanceBackground < 10) {
				isLost = true;
				outside++;
			} else {
				System.out.println("reset");
				outside = 0;
				isLost = false;
				moves.clear();
			}

			pid();

			if (outside == 10) {
				System.out.println("BEEP BEPEP");
				Sound.beep();
				findLine();
			}

		}

	}

	public void collectData() {
		currentColor = colorSensor.getColor();
		distanceLine = LinesData.calculateColorDistance(currentColor, linesData.getLine());
		distanceBackground = LinesData.calculateColorDistance(currentColor, linesData.getBackground());
		distanceMiddleLb = LinesData.calculateColorDistance(currentColor, linesData.getMiddleLineBgColor());
		distanceIntersection = LinesData.calculateColorDistance(currentColor, linesData.getIntersectionColor());
		direction = getDirection();

		// System.out.println(distanceLine + " " + distanceBackground + " " +
		// distanceMiddleLb);

	}

	public void findLine() {
		isLost = false;

		while (moves.size() > 0) {
			try {
				Move currentMove = moves.pop();
				setMotorSpeed(-currentMove.leftSpeed, -currentMove.rightSpeed);
				Delay.nsDelay(currentMove.duration);

			} catch (Exception e) {
				System.out.println("FIND LINE");
			}
		}

		while (true) {
			setMotorSpeed(tp, tp);
			collectData();
			if (getDirection() != -1) {
				break;
			}
		}

	}

	public void pid() {

		System.out.println("is lost : " + isLost);
		error = distanceMiddleLb * direction;
		derivative = error - lastError;

		turn = (kp * error) + (ki * integral) + (kd * derivative);
		recordData(kp * error, ki * integral, kd * derivative);

		int leftSpeed = (int) (tp + turn);
		int rightSpeed = (int) (tp - turn);

		setMotorSpeed(leftSpeed, rightSpeed);

		try {
			FileWriter csvWriter = new FileWriter("datata.csv", true);
			csvWriter.append("TP : TURN " + tp + " : " + turn + " SPEED ( " + leftSpeed + " : " + rightSpeed + ") \n");
			csvWriter.append(
					"ERROR : " + error + " : " + "LAST ERROR :" + lastError + " DERIVATIVE : " + derivative + "\n");
			csvWriter.append("\n");
			csvWriter.flush();
			csvWriter.close();

		} catch (IOException e) {
			// TODO Auto-generated catch block
		}

		lastError = error;

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

		/* System.out.println("DIR " + direction + " :: " + distanceBackground); */
		return direction;
	}

	public void setMotorSpeed(int leftSpeed, int rightSpeed) {

		leftSpeed = Math.min(leftSpeed, tp);
		leftSpeed = Math.max(leftSpeed, -tp);

		rightSpeed = Math.min(rightSpeed, tp);
		rightSpeed = Math.max(rightSpeed, -tp);

		motorA.setSpeed(leftSpeed);
		motorB.setSpeed(rightSpeed);

		startTime = System.nanoTime();

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

		addMove(leftSpeed, rightSpeed);
		if (moves.size() >= 2) {
			moves.get(moves.size() - 2).setDuration(startTime - endTime);
			System.out.println("hehe");
		}

		endTime = startTime;

	}

	public void addTime(long duration) {

	}

	public void addMove(int leftSpeed, int rightSpeed) {
		// System.out.println("IS LOST " + isLost);
		if (isLost) {
			System.out.println("ADDING MOVE");
			moves.add(new Move(leftSpeed, rightSpeed));
			// System.out.println("moves.add(new Move(" + leftSpeed + "," + rightSpeed +
			// "));");
		}
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

	public void recordData2(int distanceA, int distanceB, int distanceC) {
		try {
			FileWriter csvWriter = new FileWriter("data.csv", true);
			csvWriter.append("" + distanceA);
			csvWriter.append(",");
			csvWriter.append("" + distanceB);
			csvWriter.append(",");
			csvWriter.append("" + distanceC);
			csvWriter.append("\n");
			csvWriter.flush();
			csvWriter.close();

		} catch (IOException e) {
			// TODO Auto-generated catcdmh block
		}
	}

	public static void main(String[] args) throws InterruptedException {

		LineFollowerr lf = new LineFollowerr(SensorPort.S1);

		lf.start();

		/*
		 * EV3LargeRegulatedMotor motorA; EV3LargeRegulatedMotor motorB;
		 * 
		 * motorA = new EV3LargeRegulatedMotor(MotorPort.A); motorB = new
		 * EV3LargeRegulatedMotor(MotorPort.B);
		 * 
		 * Stack<Move> moves = new Stack<>();
		 * 
		 * moves.add(new Move(120, 300)); moves.add(new Move(42, 300)); moves.add(new
		 * Move(60, 300)); moves.add(new Move(78, 300)); moves.add(new Move(69, 300));
		 * moves.add(new Move(60, 300)); moves.add(new Move(69, 300)); moves.add(new
		 * Move(78, 300)); moves.add(new Move(78, 300)); moves.add(new Move(69, 300));
		 * moves.add(new Move(78, 300)); moves.add(new Move(87, 300)); moves.add(new
		 * Move(69, 300)); moves.add(new Move(69, 300)); moves.add(new Move(69, 300));
		 * moves.add(new Move(69, 300)); moves.add(new Move(69, 300)); moves.add(new
		 * Move(60, 300)); moves.add(new Move(69, 300)); moves.add(new Move(60, 300));
		 * 
		 * for (Move m : moves) { System.out.println(m.getLeftSpeed() + " " +
		 * m.getRightSpeed()); motorA.setSpeed(m.getLeftSpeed());
		 * motorB.setSpeed(m.getRightSpeed()); motorA.forward(); motorB.forward();
		 * Delay.msDelay(30); }
		 * 
		 * for (Move m : moves) { System.out.println(m.getLeftSpeed() + " " +
		 * m.getRightSpeed()); motorA.setSpeed(m.getLeftSpeed());
		 * motorB.setSpeed(m.getRightSpeed()); motorA.backward(); motorB.backward();
		 * Delay.msDelay(30); }
		 */
		/*
		 * for (int i = 0; i < 5; i++) { motorA.setSpeed(400); motorB.setSpeed(200);
		 * motorA.forward(); motorB.forward(); Delay.msDelay(100); }
		 */

	}
}
