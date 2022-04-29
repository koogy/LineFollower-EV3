
public class Move {

	int leftSpeed;
	int rightSpeed;
	long duration;

	public Move(int leftSpeed, int rightSpeed) {
		this.leftSpeed = leftSpeed;
		this.rightSpeed = rightSpeed;
	}

	public int getLeftSpeed() {
		return leftSpeed;
	}

	public long getDuration() {
		return duration;
	}

	public void setDuration(long duration) {
		this.duration = duration;
	}

	public void setLeftSpeed(int leftSpeed) {
		this.leftSpeed = leftSpeed;
	}

	public int getRightSpeed() {
		return rightSpeed;
	}

	public void setRightSpeed(int rightSpeed) {
		this.rightSpeed = rightSpeed;
	}

}
