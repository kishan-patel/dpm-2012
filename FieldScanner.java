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

	/** The US sensor is used to get distance to the light source. */
	private USSensor us;

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
	
	/**When a beacon is located, this parameter specifies whether it is in the range of us as well*/
	private boolean beaconWithinRangeOfUS = false;
	
	/**
	 * This method is periodically called when the robot is rotating 360
	 * degrees. It stores the angle at which the maximum light reading occurs
	 * while the rotation is done.
	 */
	public void timedOut() {
		currentLightReading = ls.getLightValue();
		RConsole.println("Current light reading is: "+currentLightReading);
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
		this.us = SensorAndMotorInfo.US_SENSOR;
		this.ls = SensorAndMotorInfo.BEACON_FINDER_LIGHT_SENSOR;
		this.odo = odo;
		ls.setFloodlight(true);
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
		beaconWithinRangeOfUS = false;
	}

	/**
	 * A beacon is considered to be located if the value of the max light reading while doing
	 * a 360 degree turn is above a certain treshold.
	 * @return A boolean value indicating whether the beacon was located.
	 */
	public boolean beaconLocated() {
		RConsole.println("Max light reading is: "+maxLightReading);
		if (maxLightReading > MIN_LIGHT_INTENSITY) {
			RConsole.println("Beacon detected is true.");
			return true;
		}

		return false;
	}

	/**
	 * This method is called when the beacon is located. It orients the robot towards the beacon.
	 */
	public void turnToBeacon() {
		RConsole.println("Turning to angle of max-light reading: "+angleOfMaxLightReading);
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
	 * When this method is called, the robot should have located the beacon and be facing it. If these conditions
	 * are met, this method returns whether the beacon is detected by the US sensor.
	 * @return Returns true if the distance reported by the US sensor is less than 100cm. False otherwise.
	 */
	public boolean isBeaconDetectedByUS() {
		if (us.getFilteredDistance() < 100) {
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * This method requires the robot to be facing the beacon.
	 * @return The distance to the beacon as recorded by the US sensor.
	 */
	public int getDistanceToBeacon(){
		return us.getFilteredDistance();
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
