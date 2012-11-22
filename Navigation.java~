import lejos.nxt.comm.RConsole;
import lejos.util.Timer;



public class Navigation {
	/**The allowed tolerance in the angle while rotating.*/
	private final static double ROTATION_TOLERANCE = 0.5;
	
	/**The allowed tolerance in the distance to a particular point.*/
	private final double FIANL_DISTANCE_ERROR = 2;
	
	private final double DISTANCE_ERROR_WHILE_TRAVELLING = 1;
	
	/**Speed of the motors when the robot is traveling forward.*/
	private final static int FWD_SPEED = 10;
	
	/**This number specifies how frequently correction should be done while traveling towards the light source.*/
	private final static int UPDATE_HEADING_PERIOD = 3000;
	
	/**The rotation speed.*/
	private int ROTATION_SPEED = 20;
	
	/**The forward speed.*/
	private static double FORWARD_SPEED = 8;
		
	/**An array which holds the odometry information for the robot at a given point in time.*/
	private double[] position = new double[3];
	
	/**An instance of an odometer class. It's used to get the robot's odometry information.*/
	private Odometer odo;
	
	/**Used to set the speeds of he robot.*/
	private TwoWheeledRobot robot;
	
	/**The search algorithm is used to get the tiles to move to to reach the destination*/
	private SearchAlgorithm searchAlgorithm;
	
	/**US sensor used to avoid obstacles*/
	private USSensor usSensor;
	
	
	/**Holds information about the tiles in the field.*/
	private FieldScanner fieldScanner;
	
	private static Navigation navigation = null;
	
	/**
	 * Constructor
	 */
	private Navigation(Odometer odo) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();
		this.usSensor = SensorAndMotorInfo.US_SENSOR;
		this.fieldScanner = FieldScanner.getFieldScanner(odo);
		this.fieldScanner.setNavigation(this);
	}
	

	public static Navigation getNavigation(Odometer odo){
		if(navigation==null){
			navigation = new Navigation(odo);
		}
		
		return navigation;
	}
	
	/**
	 * Robot moves forward at the present heading.
	 */
	public void goStraight(int distance){
		robot.setForwardSpeed(FWD_SPEED);
		robot.setForwardSpeed(FWD_SPEED);
		robot.distanceToRotate(convertDistance(TwoWheeledRobot.DEFAULT_LEFT_RADIUS, distance));
	}
	
	/**
	 * Robot stops any forward motion.
	 */
	public void stopGoingStraight(){
		robot.setForwardSpeed(0.0);
	}

	/**
	 * This method is called once a light source is located and causes the robot to travel towards it.
	 * While traveling towards the light source, it periodically attempts to correct the heading
	 * of the robot so that the final position of the robot is within 30 degrees of the light 
	 * source.
	 */
	public void navigateTowardsLightSource(int distanceToStopAt) {
		long start, end;//Used to keep track of when to apply the correction.
		int distanceToLightSource = usSensor.getDistance();
		start = System.currentTimeMillis();
		int noOfObjectDetections=0;
		
		while (distanceToLightSource >= distanceToStopAt||noOfObjectDetections<=5) {
			robot.setRotationSpeed(0.0);
			robot.setForwardSpeed(FWD_SPEED);
			distanceToLightSource = usSensor.getDistance();
			if(distanceToLightSource<=distanceToStopAt){
				noOfObjectDetections++;
			}else{
				noOfObjectDetections=0;
			}
			RConsole.println("Distance to light source: "
					+ distanceToLightSource);
			end = System.currentTimeMillis();
			try {//The period for the correction is specified by UPDATE_HEADING_PERIOD.
				if (end - start > UPDATE_HEADING_PERIOD) {
					fieldScanner.locateBeacon();
					fieldScanner.turnToBeacon();
					start = System.currentTimeMillis();
				} else {
					Thread.sleep(10);
				}
			} catch (InterruptedException e) {

			}
		}
		robot.setForwardSpeed(0.0);
	}
	
	public boolean isObstacleInTheWay(double x, double y){
		double distX=0;
		double distY=0;
		double theta=0;
		
		odo.getPosition(position);
		distX = x - position[0];
		distY = y - position[1];
		theta = (Math.toDegrees(Math.atan2(distX,distY)));
		theta=(theta<=0)?theta+=360:theta;
		
		//Update the heading of the robot to point to the direction of the 
		//final coordinates.
		if(Math.abs(theta-position[2])>ROTATION_TOLERANCE){
			turnTo(theta);
		}
		
		if(Math.abs(Odometer.minimumAngleFromTo(position[2], 0))<=ROTATION_TOLERANCE){
			//Obstacle may be in +ve y direction.
			if(Math.abs(odo.getYPos()-distY)>FIANL_DISTANCE_ERROR){
				return true;
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 90))<=ROTATION_TOLERANCE){
			//Obstacle may be in +ve x direction.
			if(Math.abs(odo.getXPos()-distX)>FIANL_DISTANCE_ERROR){
				return true;
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 270))<=ROTATION_TOLERANCE){
			//Obstacle may be in -ve y direction.
			if(Math.abs(odo.getYPos()-distY)>FIANL_DISTANCE_ERROR){
				return true;
			}
		}else{
			//Obstacle may be in -ve x direction.
			if(Math.abs(odo.getXPos()-distX)>FIANL_DISTANCE_ERROR){
				return true;
			}
		}
		
		return false;
		
	}
	
	public void traveToUsingSearchAlgo(double x, double y){
		double[]nextCoords;
		double nextXCoord,nextYCoord;
		do{
			odo.getPosition(position);
			nextCoords = searchAlgorithm.getNextXYCoordinate(position[0], x, position[1], y);
			nextXCoord = nextCoords[0];
			nextYCoord = nextCoords[1];
			travelToStraight(nextXCoord, nextYCoord);
			odo.getPosition(position);
		}while(Math.abs(position[0]-x)>FIANL_DISTANCE_ERROR || (Math.abs(position[1]-y))>FIANL_DISTANCE_ERROR);
		
		robot.setForwardSpeed(0.0);
	}
	
	public void travelToStraight(double x, double y){
		double distX =0;
		double distY=0;
		double theta=0;
		odo.getPosition(position);
		distX = x - position[0];
		distY = y - position[1];
		theta = (Math.toDegrees(Math.atan2(distX,distY)));
		theta=(theta<=0)?theta+=360:theta;
		
		//Update the heading of the robot to point to the direction of the 
		//final coordinates.
		if(Math.abs(theta-position[2])>ROTATION_TOLERANCE){
			turnTo(theta);
		}
		robot.setForwardSpeed(FORWARD_SPEED);
		robot.setForwardSpeed(FORWARD_SPEED);
		
		if(Math.abs(Odometer.minimumAngleFromTo(position[2], 0))<=5){
			while(Math.abs(position[1]-y)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 90))<=5){
			while(Math.abs(position[0]-x)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 180))<=5){
			while(Math.abs(position[1]-y)>DISTANCE_ERROR_WHILE_TRAVELLING){
				
				odo.getPosition(position);
			}
		}else{
			while(Math.abs(position[0]-x)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
			}
		}
	}
	
	/**	
	 * Travels to the (x,y) coordinate. Updates heading first if necessary.
	 * @param x The destination x coordinate.
	 * @param y The destination y coordinate.
	 */
	public void travelTo(double x, double y) {
		double distX =0;
		double distY=0;
		double theta=0;
		odo.getPosition(position);
		distX = x - position[0];
		distY = y - position[1];
		theta = (Math.toDegrees(Math.atan2(distX,distY)));
		theta=(theta<=0)?theta+=360:theta;
		
		//Update the heading of the robot to point to the direction of the 
		//final coordinates.
		if(Math.abs(theta-position[2])>ROTATION_TOLERANCE){
			turnTo(theta);
		}
		robot.setForwardSpeed(FORWARD_SPEED);
		robot.setForwardSpeed(FORWARD_SPEED);
		
		while(Math.abs(position[0]-x)>DISTANCE_ERROR_WHILE_TRAVELLING || (Math.abs(position[1]-y))>DISTANCE_ERROR_WHILE_TRAVELLING){
			odo.getPosition(position);
		}
		
		//Stops forward motion.
		robot.setForwardSpeed(0);
	}
	
	/**
	 * This method orients the robot to the angle supplied.
	 * @param angle The destination angle.
	 */
	public void turnTo(double angle) {
		double[] currPos = new double[3];
		double angleDiff;
		
		//Stop any forward motion.
		if (robot.getForwardSpeed() > 0.0) {
			robot.setForwardSpeed(0.0);
			robot.setForwardSpeed(0.0);
		}
		
		//Latch the initial angle and get the minimum angle to turn by to reach the destination angle.
		odo.getPosition(currPos);
		angleDiff = Odometer.minimumAngleFromTo(currPos[2], angle);
		
		if(angleDiff<0){
			robot.setRotationSpeed(-ROTATION_SPEED);
			robot.setRotationSpeed(-ROTATION_SPEED);
		}else{
			robot.setRotationSpeed(ROTATION_SPEED);	
			robot.setRotationSpeed(ROTATION_SPEED);	
		}
		while(Math.abs(angleDiff)>ROTATION_TOLERANCE){
			odo.getPosition(currPos);
			angleDiff = Odometer.minimumAngleFromTo(currPos[2], angle);
		}
			//Stop the rotation.
			robot.setRotationSpeed(0.0);
	}
	
	/**
	 * This method causes the robot to make a 360 degree turn.
	 */
	public void turn360(){
		double[] currPos = new double[3];
		double destAngle;
		double angleDiff;
		boolean startedRotation = false;
		
		//Stop any forward motion.
		if (robot.getForwardSpeed() > 0.0) {
			robot.setForwardSpeed(0.0);
		}
		
		//Latch the initial angle and get the minimum angle to turn by to reach the destination angle.
		odo.getPosition(currPos);
		destAngle = currPos[2];
		angleDiff = Odometer.minimumAngleFromTo(currPos[2], destAngle);

		robot.setRotationSpeed(ROTATION_SPEED);
		robot.setRotationSpeed(ROTATION_SPEED);
		//While loop executes until the robot hasn't rotated by 360 degrees.
		while(Math.abs(angleDiff)>ROTATION_TOLERANCE||!startedRotation){
			if(Math.abs(angleDiff)>ROTATION_TOLERANCE+5 && !startedRotation){
				startedRotation = true;
			}
			odo.getPosition(currPos);
			angleDiff = Odometer.minimumAngleFromTo(currPos[2], destAngle);
		}
		
		//Stop the rotation
		robot.setRotationSpeed(0.0);
	}
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}
