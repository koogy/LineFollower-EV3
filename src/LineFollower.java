import lejos.hardware.Button;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;

public class LineFollower {

	Car car;
	LinesData linesData;
	ColorSensor colorSensor;
	Color currentColor;

	double kp = 2.857; // TP/error
	double ki = 0;
	double kd = 0;
	double tp = 100;
	double integral = 0;
	double error = 0;
	double lastError = 0;
	double derivative = 0;
	double turn = 0;
	double dt = 1;

	public LineFollower(Port port) {
		car = new Car();
		colorSensor = new ColorSensor(port);
		linesData = new LinesData(colorSensor);

	}

	public void start() {
		Button.waitForAnyPress();
		while (Button.ESCAPE.isUp()) {

			currentColor = colorSensor.getColor();
			int distanceLine = LinesData.calculateColorDistance(currentColor, linesData.getLine());
			int distanceBackground = LinesData.calculateColorDistance(currentColor, linesData.getBackground());
			int distanceMiddleLb = LinesData.calculateColorDistance(currentColor, linesData.getMiddleLineBgColor());

			int min = Math.min(distanceLine, (Math.min(distanceBackground, distanceMiddleLb)));

			int direction = 0;
			if (min == distanceLine) {
				direction = 1;
			} else if (min == distanceBackground) {
				direction = -1;
			} else if (min == distanceMiddleLb) {
				direction = 0;
			}

			// The error tells us how far off the line's edge we are
			error = distanceMiddleLb * direction;
			derivative = error - lastError;

			turn = kp * error + ki * integral + kd * derivative;

			car.motorA.setSpeed((int) (tp + turn));
			car.motorB.setSpeed((int) (tp - turn));

			lastError = error;
		}

	}

	public void recordData(double error, double integral, double derivative) {

	}

	public static void main(String[] args) {

		LineFollower lf = new LineFollower(SensorPort.S1);
		lf.start();

	}

}
