
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.Color;

public class ColorSensor {
	EV3ColorSensor sensor;
	float[] sample;

	public ColorSensor(Port port) {
		sensor = new EV3ColorSensor(port);
		this.setRGBMode();
	}

	public EV3ColorSensor getSensor() {
		return sensor;
	}

	public int getColorID() {
		sensor.fetchSample(sample, 0);

		return (int) sample[0];
	}

	public void setRGBMode() {
		sensor.setCurrentMode("RGB");
		sample = new float[sensor.sampleSize()];
	}

	public Color getColor() {
		sensor.fetchSample(sample, 0);
		return new Color((int) (sample[0] * 255), (int) (sample[1] * 255), (int) (sample[2] * 255));
	}

	public void close() {
		sensor.close();
	}

}
