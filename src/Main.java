import lejos.hardware.port.SensorPort;

public class Main {
	public static void main(String[] args) throws InterruptedException {

		LineFollower lf = new LineFollower(SensorPort.S1);
		lf.start();

	}
}
