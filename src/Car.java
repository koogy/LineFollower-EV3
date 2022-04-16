import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class Car {

	EV3LargeRegulatedMotor motorA;
	EV3LargeRegulatedMotor motorB;
	int speed;

	public Car() {
		motorA = new EV3LargeRegulatedMotor(MotorPort.A);
		motorB = new EV3LargeRegulatedMotor(MotorPort.B);

	}

	public void driveFoward(int speed) {
		motorA.setSpeed(speed);
		motorB.setSpeed(speed);

		motorA.forward();
		motorB.forward();
	}

	public void turnLeft(int speedLeft, int speedRight, boolean tightTurn) {
		motorA.setSpeed(speedLeft);
		motorB.setSpeed(speedRight);

		if (tightTurn) {
			motorA.backward();
			motorB.forward();
		} else {
			motorA.forward();
			motorB.forward();
		}

	}

	public void turnRight(int speedLeft, int speedRight, boolean tightTurn) {
		motorA.setSpeed(speedLeft);
		motorB.setSpeed(speedRight);

		if (tightTurn) {
			motorA.forward();
			motorB.backward();
		} else {
			motorA.forward();
			motorB.forward();
		}
	}

}
