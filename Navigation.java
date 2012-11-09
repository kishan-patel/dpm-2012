

public class Navigation {
	/**The allowed tolerance in the angle while rotating.*/
	private final static double ROTATION_TOLERANCE = 0.5;
	
	/**The allowed tolerance in the distance to a particular point.*/
	private final double DISTANCE_ERROR = 2;
	
	/**Dimension of a tile*/
	private final double TILE_DIM = 30.48;
	
	/**Dimension of half a tile*/
	private final double HALF_TILE_DIM = 15.24;
	
	/**The rotation speed.*/
	private int ROTATION_SPEED = 15;
	
	/**The forward speed.*/
	private static double FORWARD_SPEED = 15;
		
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
	
	/**To prevent multiple instances of this class being created*/
	private static Navigation navigation = null;
	
	
	/**
	 * Constructor
	 */
	private Navigation(Odometer odo) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.searchAlgorithm = new SearchAlgorithm();
		this.usSensor = SensorAndMotorInfo.getUsSensor();
		this.fieldScanner = FieldScanner.getFieldScanner(odo);
	}
	
	public static Navigation getNavigation(Odometer odo){
		if(navigation == null){
			navigation = new Navigation(odo);
		}
		
		return navigation;
	}
	
	/**
	 * Robot moves forward at the present heading.
	 */
	public void goStraight(){
		robot.setRotationSpeed(0.0);
		robot.setForwardSpeed(FORWARD_SPEED);
	}
	
	/**
	 * Robot stops any forward motion.
	 */
	public void stopGoingStraight(){
		robot.setForwardSpeed(0.0);
	}
	
	public void moveToTile(int xDestTile, int yDestTile){
		int[]currentTile, destTiles;
		int nextXTile, nextYTile, distanceToObstacle;
		double distX, distY;
		boolean isObstacleInTheWay = false;
		
		do{
			currentTile = fieldScanner.getCurrentTile();
			destTiles = searchAlgorithm.getNextXYTile(currentTile[0],xDestTile,currentTile[1],yDestTile);
			nextXTile = destTiles[0];
			nextYTile = destTiles[1];
			distX = nextXTile*(TILE_DIM+HALF_TILE_DIM);
			distY = nextYTile*(TILE_DIM+HALF_TILE_DIM);
			isObstacleInTheWay = isObstacleInTheWay(distX,distY);
			if(isObstacleInTheWay){
				distanceToObstacle = usSensor.getFilteredDistance();
				fieldScanner.markObstacle(distanceToObstacle);
			}else{
				travelTo(distX,distY);
				fieldScanner.setCurrentTile(nextXTile, nextYTile);
			}
		}while((xDestTile-nextXTile)!=0||(yDestTile-nextYTile)!=0);
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
			if(Math.abs(odo.getYPos()-distY)>DISTANCE_ERROR){
				return true;
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 90))<=ROTATION_TOLERANCE){
			//Obstacle may be in +ve x direction.
			if(Math.abs(odo.getXPos()-distX)>DISTANCE_ERROR){
				return true;
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 270))<=ROTATION_TOLERANCE){
			//Obstacle may be in -ve y direction.
			if(Math.abs(odo.getYPos()-distY)>DISTANCE_ERROR){
				return true;
			}
		}else{
			//Obstacle may be in -ve x direction.
			if(Math.abs(odo.getXPos()-distX)>DISTANCE_ERROR){
				return true;
			}
		}
		
		return false;
		
	}
	
	public double[] convertTilesToDistances(int xTile, int yTile){
		double[] distances = new double[2];
		distances[0] = xTile*(TILE_DIM+HALF_TILE_DIM);
		distances[1] = yTile*(TILE_DIM+HALF_TILE_DIM);
		return distances;
	}
	
	public int[] convertDistancesToTiles(double xDist, double yDist){
		int[] tiles = new int[2];
		tiles[0] = (int)(xDist/(TILE_DIM+HALF_TILE_DIM));
		tiles[1] = (int)(yDist/(TILE_DIM+HALF_TILE_DIM));
		return tiles;
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
		
		//Causes the robot to move forward until it reaches the destination x and y coordinates.
		while(Math.abs(position[0]-x)>DISTANCE_ERROR || (Math.abs(position[1]-y))>DISTANCE_ERROR){
			odo.getPosition(position);
			robot.setForwardSpeed(FORWARD_SPEED);
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

		//While loop executes until the robot hasn't rotated by 360 degrees.
		while(Math.abs(angleDiff)>ROTATION_TOLERANCE||!startedRotation){
			if(Math.abs(angleDiff)>ROTATION_TOLERANCE+5 && !startedRotation){
				startedRotation = true;
			}
			odo.getPosition(currPos);
			angleDiff = Odometer.minimumAngleFromTo(currPos[2], destAngle);
			robot.setRotationSpeed(ROTATION_SPEED);
		}
		
		//Stop the rotation
		robot.setRotationSpeed(0.0);
	}
}
