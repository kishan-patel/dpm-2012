import lejos.nxt.LightSensor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;
import lejos.util.Timer;
import lejos.util.TimerListener;

public class FieldScanner implements TimerListener {
	/** This number represents the intensity of the light source. */
	private final static int MIN_LIGHT_INTENSITY = 40;

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
	public FieldScanner(Odometer odo) {
		this.us = SensorAndMotorInfo.getUsSensor();
		this.ls = SensorAndMotorInfo.getLeftLightSensor();
		this.odo = odo;
		this.nav = Navigation.getNavigation(odo);
		ls.setFloodlight(false);
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

	/**
	 * The goal of this method is to locate the light source. It accomplishes
	 * this by doing a rotation of 360 degrees and turning towards the light
	 * source if it is found.
	 */
	public void findLightSourceHeading() {
		reset();
		Timer timer = new Timer(10, this);
		timer.start();
		nav.turn360();
		timer.stop();
		if (maxLightReading > MIN_LIGHT_INTENSITY) {
			nav.turnTo(angleOfMaxLightReading);
		}
	}

}
