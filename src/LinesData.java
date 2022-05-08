import lejos.hardware.Button;
import lejos.robotics.Color;

public class LinesData {

	Color line;
	Color background;
	Color middleLineBgColor;
	Color intersection;

	ColorSensor colorSensor;

	public LinesData(ColorSensor sensor) {
		colorSensor = sensor;
		this.calibrate();
	}

	public void calibrate() {

		System.out.println("Press any button to measure : Line's color");
		Button.waitForAnyPress();
		line = colorSensor.getColor();

		/* line = new Color(70, 74, 14); */

		System.out.println("Press any button to measure : background's color");
		Button.waitForAnyPress();
		background = colorSensor.getColor();
		/*
		 * background = new Color(66, 128, 58);
		 */

		System.out.println("Press any button to measure : intersection's color");
		Button.waitForAnyPress();
		intersection = colorSensor.getColor();
		/*
		 * intersection = new Color(5655, 12855, 555);
		 */

		double ratioL = 0.5;
		double ratioR = 0.5;
		middleLineBgColor = new Color((int) (line.getRed() * ratioL + (int) background.getRed() * ratioR),
				(int) (line.getGreen() * ratioL + background.getGreen() * ratioR),
				(int) (line.getBlue() * ratioL + background.getBlue() * ratioR));

		this.printLinesInfo();

	}

	public void printLinesInfo() {
		System.out.println("Line : " + printColor(line));
		System.out.println("Background : " + printColor(background));
		System.out.println("Middle  : " + printColor(middleLineBgColor));
		System.out.println("Intersection  : " + printColor(intersection));

	}

	public static int calculateColorDistance(Color color1, Color color2) {

		return (int) Math.sqrt(
				Math.pow(color1.getRed() - color2.getRed(), 2) + Math.pow(color1.getGreen() - color2.getGreen(), 2)
						+ Math.pow(color1.getBlue() - color2.getBlue(), 2));

	}

	public String printColor(Color color) {
		return color.getRed() + " " + color.getGreen() + " " + color.getBlue();
	}

	public Color getLine() {
		return line;
	}

	public Color getBackground() {
		return background;
	}

	public Color getIntersectionColor() {
		return intersection;
	}

	public Color getMiddleLineBgColor() {
		return middleLineBgColor;
	}

	public ColorSensor getColorSensor() {
		return colorSensor;
	}

}
