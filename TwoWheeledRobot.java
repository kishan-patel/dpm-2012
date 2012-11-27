import lejos.nxt.NXTRegulatedMotor;

public class TwoWheeledRobot {
	/*public static final double DEFAULT_LEFT_RADIUS = 2.65;
	public static final double DEFAULT_RIGHT_RADIUS = 2.65;
	public static final double DEFAULT_WIDTH = 19;*/
	//left bigger than it should be means that the robot shifts in
	//left smaller than it should be means that the robot shifts out
	public static final double DEFAULT_LEFT_RADIUS = 2.65;
	public static final double DEFAULT_RIGHT_RADIUS = 2.65;
	public static final double DEFAULT_WIDTH = 18.5;
	public static NXTRegulatedMotor leftMotor;
	public static NXTRegulatedMotor rightMotor;
	public static double leftRadius, rightRadius, width;
	public static double forwardSpeed, rotationSpeed;
	
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor,
						   NXTRegulatedMotor rightMotor,
						   double width,
						   double leftRadius,
						   double rightRadius) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.leftRadius = leftRadius;
		this.rightRadius = rightRadius;
		this.width = width;
	}
	
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor) {
		this(leftMotor, rightMotor, DEFAULT_WIDTH, DEFAULT_LEFT_RADIUS, DEFAULT_RIGHT_RADIUS);
	}
	
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor, NXTRegulatedMotor rightMotor, double width) {
		this(leftMotor, rightMotor, width, DEFAULT_LEFT_RADIUS, DEFAULT_RIGHT_RADIUS);
	}
	
	// accessors
	public double getDisplacement() {
		return (leftMotor.getTachoCount() * leftRadius +
				rightMotor.getTachoCount() * rightRadius) *
				Math.PI / 360.0;
	}
	
	public double getHeading() {
		return (leftMotor.getTachoCount() * leftRadius -
				rightMotor.getTachoCount() * rightRadius) / width;
	}
	
	public void getDisplacementAndHeading(double [] data) {
		int leftTacho, rightTacho;
		leftTacho = -1*leftMotor.getTachoCount();
		rightTacho = -1*rightMotor.getTachoCount();
		
		data[0] = (leftTacho * leftRadius + rightTacho * rightRadius) *	Math.PI / 360.0;
		data[1] = (leftTacho * leftRadius - rightTacho * rightRadius) / width;
	}
	
	//accessors
	public double getForwardSpeed(){
		return forwardSpeed;
	}
	
	public double getRotationSpeed(){
		return rotationSpeed;
	}
	
	// mutators
	public void setForwardSpeed(double speed) {
		forwardSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}
	
	public void setRotationSpeed(double speed) {
		rotationSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}
	
	public void setSpeeds(double forwardSpeed, double rotationalSpeed) {
		double leftSpeed, rightSpeed;

		this.forwardSpeed = forwardSpeed;
		this.rotationSpeed = rotationalSpeed; 

		leftSpeed = (forwardSpeed + rotationalSpeed * width * Math.PI / 360.0) *
				180.0 / (leftRadius * Math.PI);
		rightSpeed = (forwardSpeed - rotationalSpeed * width * Math.PI / 360.0) *
				180.0 / (rightRadius * Math.PI);

		// set motor directions
		if (leftSpeed > 0.0){
			//leftMotor.forward();
			leftMotor.backward();
			leftSpeed = -leftSpeed;
			}
		else {
			//leftMotor.backward();
			//leftSpeed = -leftSpeed;
			leftMotor.forward();
			
		}
		
		if (rightSpeed > 0.0){
			//rightMotor.forward();
			rightMotor.backward();
			rightSpeed = -rightSpeed;
			}
		else {
			/*rightMotor.backward();
			rightSpeed = -rightSpeed;*/
			rightMotor.forward();
			
		}
		
		// set motor speeds
		if (leftSpeed > 900.0)
			leftMotor.setSpeed(900);
		else
			leftMotor.setSpeed((int)leftSpeed);
		
		if (rightSpeed > 900.0)
			rightMotor.setSpeed(900);
		else
			rightMotor.setSpeed((int)rightSpeed);
	}
	
	public void distanceToRotate(int distance){
		// drive forward two tiles
		leftMotor.rotate(distance, true);
		rightMotor.rotate(distance, false);
	}
}
