import lejos.robotics.Color;

public class LinesData {

	Color line;
	Color background;
	Color middleLineBgColor;

	ColorSensor colorSensor;

	public LinesData(ColorSensor sensor) {
		colorSensor = sensor;
		this.calibrate();
		/* this.printLinesInfo(); */
	}

	public void calibrate() {
		System.out.println("--------------");
		System.out.println("Calibrating...");
		System.out.println("--------------");

		System.out.println("Press any button to measure : Line's color");
		/*
		 * Button.waitForAnyPress(); line = colorSensor.getColor();
		 */
		line = new Color(65, 70, 15);

		System.out.println("Press any button to measure : background's color");
		/*
		 * Button.waitForAnyPress(); background = colorSensor.getColor();
		 */
		background = new Color(62, 125, 56);
		double ratioL = 0.7;
		double ratioR = 0.3;
		middleLineBgColor = new Color((int) (line.getRed() * ratioL + (int) background.getRed() * ratioR),
				(int) (line.getGreen() * ratioL + background.getGreen() * ratioR),
				(int) (line.getBlue() * ratioL + background.getBlue() * ratioR));

		printLinesInfo();
		/*
		 * middleLineBgColor = new Color((line.getRed() + background.getRed()) / 2,
		 * (line.getGreen() + background.getGreen()) / 2, (line.getBlue() +
		 * background.getBlue()) / 2); printLinesInfo();
		 */

		System.out.println(LinesData.calculateColorDistance(line, middleLineBgColor));
		System.out.println(LinesData.calculateColorDistance(background, middleLineBgColor));

	}

	public void printLinesInfo() {
		System.out.println("Line : " + getColors(line));
		System.out.println("Background : " + getColors(background));
		System.out.println("Middle  : " + getColors(middleLineBgColor));

	}

	public static int calculateColorDistance(Color color1, Color color2) {

		return (int) Math.sqrt(
				Math.pow(color1.getRed() - color2.getRed(), 2) + Math.pow(color1.getGreen() - color2.getGreen(), 2)
						+ Math.pow(color1.getBlue() - color2.getBlue(), 2));

	}

	public String getColors(Color color) {
		return color.getRed() + " " + color.getGreen() + " " + color.getBlue();
	}

	public Color getLine() {
		return line;
	}

	public Color getBackground() {
		return background;
	}

	public Color getMiddleLineBgColor() {
		return middleLineBgColor;
	}

	public ColorSensor getColorSensor() {
		return colorSensor;
	}

}
