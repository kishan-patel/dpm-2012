import lejos.nxt.LightSensor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class FieldScanner implements TimerListener {
	/** This number represents the intensity of the light source. */
	private final static int MIN_LIGHT_INTENSITY = 40;
	
	/**The tolerance allowed in the angle reading*/
	private final int ANGLE_TOLERANCE = 2;
	
	/**Constant to represent the status of a tile.*/
	private final int UNKNOWN = 0;
	
	/**Constant to represent the status of a tile.*/
	private final int EMPTY = 1;
	
	/**Constant to represent the status of a tile*/
	private final int OBSTACLE = 2;
	
	/**Constant to represent the status of a tile*/
	private final int BEACON = 3;
	
	/**
	 * An instance of an odometer class. It's used to get the robot's odometry
	 * information.
	 */
	private Odometer odo;

	/** The US sensor is used to get distance to the light source. */
	private UltrasonicSensor us;

	/** The light sensor is used to locate the light source. */
	private LightSensor ls;

	/** Used to control the forward motion and rotation of the robot. */
	private Navigation nav;

	/**
	 * Used to keep a track of the maximum light reading encountered when making
	 * a 360 turn.
	 */
	private int maxLightReading = 0;

	/** Instantaneous light reading. */
	private int currentLightReading = 0;

	/** The heading at which the maximum light reading occurred. */
	private double angleOfMaxLightReading = 0;

	/**
	 * An array which holds the odometry information for the robot at a given
	 * point in time.
	 */
	private double[] pos = new double[3];

	/**
	 * Contains information about the tiles in the field. A status of a tile can be unexplored,
	 * empty, obstacle, or beacon.
	 */
	private static int[][] fieldInfo = new int[20][20];
	
	/**Ensures only one instance of this class is created*/
	private static FieldScanner fieldScanner = null;
	
	/**Stores the x and y positions of the current tile.*/
	private  int[] currentTile = new int[2];
	
	
	/**
	 * This method is periodically called when the robot is rotating 360
	 * degrees. It stores the angle at which the maximum light reading occurs
	 * while the rotation is done.
	 */
	public void timedOut() {
		currentLightReading = ls.getLightValue();
		if (currentLightReading > maxLightReading) {
			maxLightReading = currentLightReading;
			odo.getPosition(pos);
			angleOfMaxLightReading = pos[2];
			RConsole.println("(max. light reading,angle)=(" + maxLightReading
					+ "," + pos[2] + ")");
			Sound.beep();
		}
	}

	/**
	 * Constructor
	 */
	private FieldScanner(Odometer odo) {
		this.us = SensorAndMotorInfo.getUsSensor();
		this.ls = SensorAndMotorInfo.getLeftLightSensor();
		this.odo = odo;
		ls.setFloodlight(false);
	}
	
	public static FieldScanner getFieldScanner(Odometer odo){
		if(fieldScanner == null){
			fieldScanner = new FieldScanner(odo);
		}
		
		return fieldScanner;
	}
	
	/**
	 * This method is called when the timer restarts (i.e. when the robot
	 * commences to do a 360 degree rotation to locate the light source).
	 */
	public void reset() {
		maxLightReading = 0;
	}

	/**
	 * 
	 */
	public boolean LightSourceFound() {
		if (maxLightReading > MIN_LIGHT_INTENSITY) {
			return true;
		}

		return false;
	}

	public void turnToLightSourceHeading() {
		nav.turnTo(angleOfMaxLightReading);
	}

	public void locateLightSource() {
		reset();
		Timer timer = new Timer(10, this);
		timer.start();
		nav.turn360();
		timer.stop();
	}

	public void setNavigation(Navigation navigation){
		nav = navigation;
	}
	
	public void markObstacle(int distanceToObstacle){
		double[] pos = new double[3];
		odo.getPosition(pos);
		double currentXDist = pos[0];
		double currentYDist = pos[1];
		double currentTheta = pos[2];
		int[] tiles = new int[2];

		if(Math.abs(Odometer.minimumAngleFromTo(currentTheta, 0))<=ANGLE_TOLERANCE){
			//Obstacle is in +ve y direction.
			tiles = nav.convertDistancesToTiles(currentXDist, currentYDist+distanceToObstacle);
		}else if (Math.abs(Odometer.minimumAngleFromTo(currentTheta, 90))<=ANGLE_TOLERANCE){
			//Obstacle is in +ve x direction.
			tiles = nav.convertDistancesToTiles(currentXDist+distanceToObstacle, currentYDist);
		}else if (Math.abs(Odometer.minimumAngleFromTo(currentTheta, 270))<=ANGLE_TOLERANCE){
			//Obstacle is in -ve y direction.
			tiles = nav.convertDistancesToTiles(currentXDist, currentYDist-distanceToObstacle);
		}else{
			//Obstacle is in -ve x direction.
			tiles = nav.convertDistancesToTiles(currentXDist-distanceToObstacle, currentYDist);
		}
		
		fieldInfo[tiles[0]][tiles[1]] = OBSTACLE;
	}
	
	public int[] getCurrentTile(){
		return currentTile;
	}
	
	public void setCurrentTile(int xCurrentTile, int yCurrentTile){
		currentTile[0] = xCurrentTile;
		currentTile[1] = yCurrentTile;
	}
	
	public static int[][] getFieldInfo(){
		return fieldInfo;
	}
}
