import bluetooth.PlayerRole;
import lejos.nxt.comm.RConsole;
import lejos.util.Timer;



public class Navigation {
	/**The allowed tolerance in the angle while rotating.*/
	private final static double ROTATION_TOLERANCE = 1;
	
	/**The allowed tolerance in the distance to a particular point.*/
	private final double FIANL_DISTANCE_ERROR = 2;
	
	private final double DISTANCE_ERROR_WHILE_TRAVELLING = 1;
	
	/**Speed of the motors when the robot is traveling forward.*/
	private final static int FWD_SPEED = 5;
	
	/**This number specifies how frequently correction should be done while traveling towards the light source.*/
	private final static int UPDATE_HEADING_PERIOD = 3000;
	
	/**The rotation speed.*/
	private int ROTATION_SPEED = 20;
	
	/**The forward speed.*/
	private static double FORWARD_SPEED = 5;
		
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
	
	private boolean obstacleDetected = false;
	private boolean beaconDetected = false;
	public boolean carryingBeacon = false;
	LightLocalizer ll;
	public static Coordinates initPoint = new Coordinates();
	private OdoCorrection odoCorrection;
	private boolean travellingDuringObstacleAvoidance = false;
	
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
		this.ll =   new LightLocalizer(odo, SensorAndMotorInfo.LS_LOCALIZATION_SENSOR);
		this.odoCorrection = new OdoCorrection(odo);
		
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
	
	
	public void travelToInXandY(double x, double y){
		double[]nextCoords;
		double nextXCoord,nextYCoord;
		
		do{
			odo.getPosition(position);
			nextCoords = searchAlgorithm.getNextXYCoordinate(position[0], x, position[1], y);
			nextXCoord = nextCoords[0];
			nextYCoord = nextCoords[1];
			if(!travellingDuringObstacleAvoidance){
				travelToStraight(nextXCoord, nextYCoord);
			}else{
				travelToStraightNoObstacle(nextXCoord, nextYCoord);
			}
			
			//If the current position that we are attempting to get at is blocked, we stop trying to
			//go there.
			if(obstacleDetected || beaconDetected){
				obstacleDetected = false;
				beaconDetected = false;
				break;
			}
			odo.getPosition(position);
		}while(Math.abs(position[0]-x)>FIANL_DISTANCE_ERROR || (Math.abs(position[1]-y))>FIANL_DISTANCE_ERROR);
		
		robot.setForwardSpeed(0.0);
	}
	
	public void travelToStraight(double x, double y){
		double distX =0;
		double distY=0;
		double theta=0;
		int noOfObjectDetections=0;
		int distanceToObstacle = usSensor.getDistance();
		obstacleDetected = false;
		
		//Calculates the distance to travel to reach the destination as well as the angle to turn to.
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
		
		initPoint = Odometer.getCoordinates();
		Timer timer = new Timer(10, odoCorrection);
		timer.start();
		//Start going forward.
		if(Math.abs(Odometer.minimumAngleFromTo(position[2], 0))<=5){
			while(Math.abs(position[1]-y)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 0))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(0);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}
				distanceToObstacle = usSensor.getDistance();
				RConsole.println("Distance to obs. "+distanceToObstacle);
				if(distanceToObstacle<=30.48){
					noOfObjectDetections++;
				}else{
					noOfObjectDetections = 0;
				}
				
				if(distanceToObstacle<30.48 && noOfObjectDetections>5){
					RConsole.println("Distance to obstacle: "+distanceToObstacle);
					RConsole.println("Current y position: "+position[1]);
					RConsole.println("Distace to dest."+Math.abs(MainMaster.dyCoordinate-(position[1]+distanceToObstacle+20)));
					if(Math.abs(MainMaster.dyCoordinate-(position[1]+distanceToObstacle+22))<=10){
						if(!carryingBeacon){
							beaconDetected = true;
							break;
						}
					}else{
						obstacleDetected = true;
						break;
					}
				}
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 90))<=5){
			while(Math.abs(position[0]-x)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 90))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(90);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}
				distanceToObstacle = usSensor.getDistance();
				RConsole.println("Distance to obs. "+distanceToObstacle);
				if(distanceToObstacle<=30.48){
					noOfObjectDetections++;
				}else{
					noOfObjectDetections = 0;
				}
				
				if(distanceToObstacle<30.48 && noOfObjectDetections>5){
					RConsole.println("Distance to obstacle: "+distanceToObstacle);
					RConsole.println("Distace to dest."+Math.abs(MainMaster.dxCoordinate-(position[0]+distanceToObstacle+20)));
					if(Math.abs(MainMaster.dxCoordinate-(position[0]+distanceToObstacle+22))<=10){
						if(!carryingBeacon){
							beaconDetected = true;
							break;
						}
					}else{
						obstacleDetected = true;
						break;
					}
				}
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 180))<=5){
			while(Math.abs(position[1]-y)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 180))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(180);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}
				distanceToObstacle = usSensor.getDistance();
				RConsole.println("Distance to obs. "+distanceToObstacle);
				if(distanceToObstacle<=30.48){
					noOfObjectDetections++;
				}else{
					noOfObjectDetections = 0;
				}
				
				if(distanceToObstacle<30.48 && noOfObjectDetections>5){
					RConsole.println("Distance to obstacle: "+distanceToObstacle);
					RConsole.println("Distace to dest."+Math.abs(MainMaster.dyCoordinate-(position[1]+distanceToObstacle)));
					if(Math.abs(MainMaster.dyCoordinate-(position[1]+distanceToObstacle+22))<=10){
						if(!carryingBeacon){
							beaconDetected = true;
							break;
						}
					}else{
						obstacleDetected = true;
						break;
					}
				}
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 270))<=5){
			while(Math.abs(position[0]-x)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 270))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(270);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}
				distanceToObstacle = usSensor.getDistance();
				RConsole.println("Distance to obs. "+distanceToObstacle);
				if(distanceToObstacle<=30.48){
					noOfObjectDetections++;
				}else{
					noOfObjectDetections = 0;
				}
				
				if(distanceToObstacle<30.48 && noOfObjectDetections>5){
					RConsole.println("Distance to obstacle: "+distanceToObstacle);
					RConsole.println("Distace to dest."+Math.abs(MainMaster.dxCoordinate-(position[0]+distanceToObstacle)));
					if(Math.abs(MainMaster.dxCoordinate-(position[0]+distanceToObstacle+22))<=10){
						if(!carryingBeacon){
							beaconDetected = true;
							break;
						}
					}else{
						obstacleDetected = true;
						break;
					}
				}
			}
		}
		
		timer.stop();
		RConsole.println("Stopped 2nd timer");
		robot.setForwardSpeed(0);
		robot.setForwardSpeed(0);

		if(obstacleDetected){
			if(Math.abs(position[0]-x)<=30.48 && Math.abs(position[1]-y)<=30.48){
				if(MainMaster.role == PlayerRole.ATTACKER){
					searchAlgorithm.markCurrentAttackerLocationBlocked();
				}else{
					searchAlgorithm.markCurrentDefenderLocationAsBlocked();
				}
			}
			
			avoidObstacle();
		}
	}
	
	
	public void travelToStraightNoObstacle(double x, double y){
		double distX =0;
		double distY=0;
		double theta=0;		
		obstacleDetected = false;
		
		//Calculates the distance to travel to reach the destination as well as the angle to turn to.
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
		
		//Timer to do angular correction.
		Timer timer = new Timer(10, odoCorrection);
		timer.start();
		
		//Start going forward.
		if(Math.abs(Odometer.minimumAngleFromTo(position[2], 0))<=5){
			while(Math.abs(position[1]-y)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 0))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(0);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 90))<=5){
			while(Math.abs(position[0]-x)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 90))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(90);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}				
			}
		}else if (Math.abs(Odometer.minimumAngleFromTo(position[2], 180))<=5){
			while(Math.abs(position[1]-y)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 180))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(180);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}
			}
		}else{
			while(Math.abs(position[0]-x)>DISTANCE_ERROR_WHILE_TRAVELLING){
				odo.getPosition(position);
				if(Math.abs(Odometer.minimumAngleFromTo(odo.getTheta(), 270))>=1){
					RConsole.println("Applying theta correction");
					RConsole.println("Angle before correcting is: "+odo.getTheta());
					timer.stop();
					RConsole.println("Stopped first timer");
					turnTo(270);
					timer = new  Timer(10,odoCorrection);
					timer.start();
					robot.setForwardSpeed(FORWARD_SPEED);
					robot.setForwardSpeed(FORWARD_SPEED);
				}
				try{Thread.sleep(10);}catch(InterruptedException e){}
			}
		}
		
		timer.stop();
		robot.setForwardSpeed(0);
		robot.setForwardSpeed(0);

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
		//angleDiff = getCorrectionAngle(angle);
		/*if(angleDiff<0){
			robot.setRotationSpeed(-ROTATION_SPEED);
			robot.setRotationSpeed(-ROTATION_SPEED);
		}else{
			robot.setRotationSpeed(ROTATION_SPEED);	
			robot.setRotationSpeed(ROTATION_SPEED);	
		}
		while(Math.abs(angleDiff)>ROTATION_TOLERANCE){
			odo.getPosition(currPos);
			angleDiff = Odometer.minimumAngleFromTo(currPos[2], angle);
		}*/
		// turn 90 degrees clockwise
			/*if(angleDiff<0){
					TwoWheeledRobot.leftMotor.setSpeed(-ROTATION_SPEED);
					TwoWheeledRobot.rightMotor.setSpeed(-ROTATION_SPEED);
			}else{
				TwoWheeledRobot.leftMotor.setSpeed(ROTATION_SPEED);
				TwoWheeledRobot.rightMotor.setSpeed(ROTATION_SPEED);
			}*/
		
			while(Math.abs(angleDiff)>ROTATION_TOLERANCE){
				TwoWheeledRobot.leftMotor.setSpeed(100);
				TwoWheeledRobot.rightMotor.setSpeed(100);
				TwoWheeledRobot.rightMotor.rotate(convertAngle(TwoWheeledRobot.rightRadius, TwoWheeledRobot.DEFAULT_WIDTH, angleDiff), true);
				TwoWheeledRobot.leftMotor.rotate(-convertAngle(TwoWheeledRobot.leftRadius, TwoWheeledRobot.DEFAULT_WIDTH, angleDiff), false);
				angleDiff = Odometer.minimumAngleFromTo(odo.getTheta(), angle);
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

		/*robot.setRotationSpeed(ROTATION_SPEED);
		robot.setRotationSpeed(ROTATION_SPEED);
		//While loop executes until the robot hasn't rotated by 360 degrees.
		while(Math.abs(angleDiff)>ROTATION_TOLERANCE||!startedRotation){
			if(Math.abs(angleDiff)>ROTATION_TOLERANCE+5 && !startedRotation){
				startedRotation = true;
			}
			odo.getPosition(currPos);
			angleDiff = Odometer.minimumAngleFromTo(currPos[2], destAngle);
		}*/
		angleDiff = 5;
		while(Math.abs(angleDiff)>ROTATION_TOLERANCE){
			TwoWheeledRobot.leftMotor.setSpeed(100);
			TwoWheeledRobot.rightMotor.setSpeed(100);
			TwoWheeledRobot.rightMotor.rotate(convertAngle(TwoWheeledRobot.rightRadius, TwoWheeledRobot.DEFAULT_WIDTH, 360), true);
			TwoWheeledRobot.leftMotor.rotate(-convertAngle(TwoWheeledRobot.leftRadius, TwoWheeledRobot.DEFAULT_WIDTH, 360), false);
			angleDiff = Odometer.minimumAngleFromTo(odo.getTheta(), destAngle);
		}
		
		//Stop the rotation
		robot.setRotationSpeed(0.0);
	}
	
	/**
	 * This method is going to find an alternative route to pass through an obstacle
	 */
	public void avoidObstacle(){
				
		stopGoingStraight();
		this.travellingDuringObstacleAvoidance = true;
		int maxSensor = 30;
		int sensorAverage = 0;
		double bearing = odo.getTheta();		
		int count = 10;
		double x = odo.getXPos();
		double y = odo.getYPos();
		
		// CHECK FIRST IF IT'S A WALL
		
		// Case for wall at the left side
		if( x < 30 && bearing > 250 && bearing < 290 ){
			
		}else if( x > 275 && bearing > 70 && bearing < 110 ){// Case for wall at the right side
			
		}else if( y < 30 && bearing > 160 && bearing < 200 ){// Case for wall at the bottom side
			
		}else if( y > 275 && (bearing > 340 || bearing < 20) ){// Case for wall at the upper side
			
		}else{// Case for it's really an obstacle
		
		if( bearing > 340 || bearing < 20 ){
		// Turning to the right and check availability
		
		bearing = bearing + 90;		
		turnTo(bearing);		
		
		int i = 0;
		while( i < count ){
			
			sensorAverage = sensorAverage + usSensor.getDistance();
			i++;
		}
		sensorAverage = sensorAverage/count;
		
		if( sensorAverage > maxSensor ){
			// Go straight if there is no obstacle			
			
			obstacleTravel(30.48);
			
			// Turn to the left 			
			bearing = bearing - 90;			
			turnTo(bearing);			
			
			
			
		}
		else{
		// Right is occupied
		// Turning to the left and check availability		
		
		bearing = bearing - 180;		
		turnTo(bearing);	
			
		i = 0;
		sensorAverage = 0;
		while( i < count ){
				
			sensorAverage = sensorAverage + usSensor.getDistance();
			i++;	
		}
		sensorAverage = sensorAverage/count;
		
		if( sensorAverage > maxSensor ){		
			// Go straight if there is no obstacle
			
			
			obstacleTravel(30.48);
					
			// Turn to the right			
			bearing = bearing + 90;			
			turnTo(bearing);
			
			
			
		}else{
			// Left is occupied
			// Turning left to face backward		
			bearing = bearing - 90;		
			turnTo(bearing);
			
		}
			
		}
		
		}else{
			

			// Turning to the left and check availability			
			bearing = bearing - 90;		
			turnTo(bearing);		
			
			int i = 0;
			while( i < count ){
				
				sensorAverage = sensorAverage + usSensor.getDistance();
				i++;
			}
			sensorAverage = sensorAverage/count;
			
			if( sensorAverage > maxSensor ){
				// Go straight if there is no obstacle			
				
				obstacleTravel(30.48);
				
				// Turn to the right 			
				bearing = bearing + 90;			
				turnTo(bearing);			
				
				
				
			}
			else{
			// Left is occupied
			// Turning to the right and check availability		
			
			bearing = bearing + 180;		
			turnTo(bearing);	
				
			i = 0;
			sensorAverage = 0;
			while( i < count ){
					
				sensorAverage = sensorAverage + usSensor.getDistance();
				i++;	
			}
			sensorAverage = sensorAverage/count;
			
			if( sensorAverage > maxSensor ){		
				// Go straight if there is no obstacle				
				obstacleTravel(30.48);
						
				// Turn to the left			
				bearing = bearing - 90;			
				turnTo(bearing);
				
				
				
			}else{
				// Left is occupied
				// Turning right to face backward		
				bearing = bearing + 90;		
				turnTo(bearing);
				
			}
				
			}
			
		}
		
		}
		this.travellingDuringObstacleAvoidance = false;
	}
	
	/**
	 * This method is going to move the robot forward according to the distance(Used primarily for obstacle avoider)
	 * @param distance
	 */
	private void obstacleTravel(double distance){
		
		double bearing = odo.getTheta();
		
		// Positive y
		if( bearing < 20 || bearing > 340 ){
			travelToInXandY(odo.getXPos(), odo.getYPos() + distance);
			
		// Positive x
		}else{if( bearing > 70 && bearing < 110 ){
			travelToInXandY(odo.getXPos() + distance, odo.getYPos());
			
		// Negative y	
		}else{if( bearing > 160 && bearing < 200 ){
			travelToInXandY(odo.getXPos(), odo.getYPos() - distance);
			
		// Negative x	
		}else{
			travelToInXandY(odo.getXPos() - distance, odo.getYPos());	
		}			
		}			
		}
		
		
	}

	/**
	 * 
	 * @param radius
	 * @param width
	 * @param angle
	 * @return the amount of degrees a motor should turn to make the robot turn a certain angle
	 */
	public static int convertAngle(double radius, double width, double angle) {
		return (int) ((width * angle) / (radius * 2.0));
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
}
