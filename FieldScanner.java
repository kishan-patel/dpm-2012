import lejos.nxt.LightSensor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class FieldScanner implements TimerListener {
	/** This number represents the intensity of the light source. */
	private final static int MIN_LIGHT_INTENSITY = 34;
	
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
	
	/** Used to control the forward motion and rotation of the robot. */
	private Navigation nav;

	/**
	 * Used to keep a track of the maximum light reading encountered when making
	 * a 360 turn.
	 */
	private int maxLightReading = 0;

	private int distanceToMaxLightReading = 0;
	
	/** Instantaneous light reading. */
	private int currentLightReading = 0;

	private int currentDistanceToReading = 0;
	
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
	
	/**When a beacon is located, this parameter specifies whether it is in the range of us as well*/
	private boolean beaconWithinRangeOfUS = false;
	
	/**
	 * This method is periodically called when the robot is rotating 360
	 * degrees. It stores the angle at which the maximum light reading occurs
	 * while the rotation is done.
	 */
	public void timedOut() {
		currentLightReading = LightFilter.getBeaconLight();
		currentDistanceToReading = USFilter.getUS();
		RConsole.println("Current light reading is: "+currentLightReading);
		RConsole.println("Current distance reading is: "+currentDistanceToReading);
		if (currentLightReading > maxLightReading && ((currentLightReading > Constants.LV_AT_30 && currentDistanceToReading<=35)||(currentLightReading > Constants.LV_AT_60 && currentDistanceToReading>=35))) {
			maxLightReading = currentLightReading;
			distanceToMaxLightReading = currentDistanceToReading;
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
		this.odo = odo;
		SensorAndMotorInfo.BEACON_FINDER_LIGHT_SENSOR.setFloodlight(false);
	}
	
	/**
	 * Ensures that a single instance of FieldScanner is exists.
	 * @param odo The odometer object associated with this robot.
	 * @return A FieldScanner object.
	 */
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
		distanceToMaxLightReading = 0;
		beaconWithinRangeOfUS = false;
	}

	/**
	 * A beacon is considered to be located if the value of the max light reading while doing
	 * a 360 degree turn is above a certain treshold.
	 * @return A boolean value indicating whether the beacon was located.
	 */
	public boolean beaconLocated() {
		RConsole.println("Max light reading is: "+maxLightReading);
		if ((maxLightReading > Constants.LV_AT_30 && currentDistanceToReading<=40)||(maxLightReading > Constants.LV_AT_60 && currentDistanceToReading<=60)) {
			return true;
		}

		return false;
	}

	/**
	 * This method is called when the beacon is located. It orients the robot towards the beacon.
	 */
	public void turnToBeacon() {
		nav.turnTo(angleOfMaxLightReading);
	}

	/**
	 * This method does a 360 degree turn to locate the beacon.
	 */
	public void locateBeacon() {
		reset();
		Timer timer = new Timer(10, this);
		timer.start();
		nav.turn360();
		timer.stop();
	}
	
	/**
	 * Provides the FieldScanner class access to the navigation object.
	 * @param navigation The navigation object associated with the robot.
	 */
	public void setNavigation(Navigation navigation){
		nav = navigation;
	}
	
	public int getMaxLightReading(){
		return maxLightReading;
	}
}
