import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;

public class USLocalizer {
	/**The type of location techniques that are available.*/
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	/**The speed of the motors while rotating.*/
	public static double ROTATION_SPEED = 20;
	/**Value used for the rising_edge.*/
	public static int TRESH_HOLD = 7;
	/**Specifies the distance to the wall that confirms it has been detected.*/
	public static int WALL_DIST = 40;
	/**The noise margin.*/
	public static int NOISE = 1;
	/**Used to control the forward motion and rotation of the robot.*/
	private Navigation nav;
	/**An instance of an odometer class. It's used to get the robot's odometry information.*/
	private Odometer odo;
	/**Used to set the speeds of he robot.*/
	private TwoWheeledRobot robot;
	/**The US sensor is used to get distance to the light source.*/
	private USSensor us;
	/**The type of location technique to employ.*/
	private LocalizationType locType;
	/**The number of times a wall is detected.*/
	private int objDetectCount=0;
	/**The number of times a wall is not detected.*/
	private int noObjDetectCount=0;
	
	/**
	 * Constructor. 
	 */
	public USLocalizer(Odometer odo, USSensor us, LocalizationType locType) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.us = us;
		this.locType = locType;
		RConsole.println("About to get navigation object");
		this.nav = Navigation.getNavigation(odo);
		// switch off the ultrasonic sensor
		//us.off();
	}
	
	/**
	 * Localizes the robot using either the falling_edge technique or the rising_edge technique.
	 */
	public void doLocalization() {
		double [] pos = new double [3];
		double [] noisePos = new double [3];
		double angleA, angleB,angle;
		int distance;
		if (locType == LocalizationType.FALLING_EDGE) {
			// rotate the robot until it sees no wall
			while((distance=getFilteredData()) <= WALL_DIST||noObjDetectCount<5){
				RConsole.println(""+distance);
				RConsole.println("no obj. detect count: "+noObjDetectCount);
				robot.setRotationSpeed(ROTATION_SPEED);
			}
			RConsole.println("Robot sees no wall");
			
			// keep rotating until the robot sees a wall, then latch the angle. First outside the noise
			// margin, then inside.
			while((distance=getFilteredData())>WALL_DIST + NOISE||objDetectCount<5){
				RConsole.println(""+distance);
				RConsole.println("obj. detect count: "+objDetectCount);
				robot.setRotationSpeed(ROTATION_SPEED);
			}
			odo.getPosition(noisePos);	
			while((distance=getFilteredData())>=WALL_DIST - NOISE||objDetectCount<5){
				RConsole.println(""+distance);
				RConsole.println("obj. detect count: "+objDetectCount);
				robot.setRotationSpeed(ROTATION_SPEED);
			}
			Sound.beep();
			odo.getPosition(pos);
			angleA = (pos[2]+noisePos[2])/2;
			RConsole.println("Robot saw a wall withing the noise margin");
			
			// switch direction and wait until it sees no wall
			while((distance=getFilteredData())<WALL_DIST||noObjDetectCount<5){
				RConsole.println(""+distance);
				RConsole.println("no obj. detect count: "+noObjDetectCount);
				robot.setRotationSpeed(-ROTATION_SPEED);
			}
			RConsole.println("Robot sees no wall");
			
			// keep rotating until the robot sees a wall, then latch the angle. First outside the noise
			// margin, then inside.
			while((distance=getFilteredData())>WALL_DIST+NOISE||objDetectCount<5){
				RConsole.println(""+distance);
				RConsole.println("obj. detect count: "+objDetectCount);
				robot.setRotationSpeed(-ROTATION_SPEED);
			}
			odo.getPosition(noisePos);
			while((distance=getFilteredData())>=WALL_DIST-NOISE||objDetectCount<5){
				RConsole.println(""+distance);
				RConsole.println("obj. detect count: "+objDetectCount);
				robot.setRotationSpeed(-ROTATION_SPEED);
			}
			Sound.beep();
			odo.getPosition(pos);
			angleB = (pos[2]+noisePos[2])/2;
			RConsole.println("Robot saw a wall within the noise margin");
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			if ( angleA > angleB){ 
				angle = 45 - (angleA+angleB)/2;
			}else{
				angle = 225 - (angleA+angleB)/2;
			}
			
			odo.setPosition(new double [] {0.0, 0.0, angleB+angle}, new boolean [] {true, true, true});
			nav.turnTo(0.0);
			odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
			robot.setRotationSpeed(0);
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			//
			// FILL THIS IN
			//
			// rotate the robot until it sees a wall.
			while(getFilteredData()>WALL_DIST - TRESH_HOLD){
				robot.setRotationSpeed(ROTATION_SPEED);
			}
			
			//keep rotating until the robot sees no wall, then latch the angle.
			while(getFilteredData()<=WALL_DIST - NOISE){
				robot.setRotationSpeed(ROTATION_SPEED);
			}
			Sound.beep();
			odo.getPosition(noisePos);
			angleB = (pos[2]+noisePos[2])/2;
			
			//switch direction and wait until it sees a wall
			while(getFilteredData() > WALL_DIST - TRESH_HOLD){
				robot.setRotationSpeed(-ROTATION_SPEED);
			}
			
			//keep rotating until the robot sees no wall, then latch the angle
			while(getFilteredData()< WALL_DIST - NOISE){
				robot.setRotationSpeed(-ROTATION_SPEED);
			}
			odo.getPosition(noisePos);
			while(getFilteredData()<=WALL_DIST + NOISE){
				robot.setRotationSpeed(-ROTATION_SPEED);
			}
			
			Sound.beep();
			odo.getPosition(pos);
			angleA = (noisePos[2]+pos[2])/2;
			if(angleA < angleB){
				angle = 45 - (angleA+angleB)/2;
			}else{
				angle = 255 - (angleA + angleB)/2;
			}
			
			// update the odometer position (example to follow:)
			odo.setPosition(new double [] {0.0, 0.0, angleA+angle }, new boolean [] {false, false, true});
			robot.setRotationSpeed(0);
			
		}
	}
	
	/**
	 * Filters the distance reported by the US sensor.
	 * @return The filtered distance.
	 */
	private int getFilteredData() {
		int distance;
		distance = us.getDistance();
		try{Thread.sleep(10);}catch(InterruptedException e){}
		if(distance > WALL_DIST){
			noObjDetectCount++;
			objDetectCount = 0;
		}else{
			objDetectCount++;
			noObjDetectCount = 0;
		}
		return distance;
	}

}
