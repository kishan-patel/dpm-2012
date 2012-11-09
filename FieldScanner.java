import lejos.nxt.LightSensor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;
import lejos.util.Timer;
import lejos.util.TimerListener;
public class FieldScanner implements TimerListener {
	/**This number specifies how frequently correction should be done while traveling towards the light source.*/
	private final static int UPDATE_HEADING_PERIOD = 3000;
	/**This number represents the intensity of the light source.*/
	private final static int MIN_LIGHT_INTENSITY = 40;
	/**Describes how far from the light source the robot should stop.*/
	private final static int MIN_DIS_TO_LS = 20;
	/**An instance of an odometer class. It's used to get the robot's odometry information.*/
	private Odometer odo;
	/**The US sensor is used to get distance to the light source.*/
	private UltrasonicSensor us;
	/**The light sensor is used to locate the light source.*/
	private LightSensor ls;
	/**Used to control the forward motion and rotation of the robot.*/
	private Navigation nav;
	/**Boolean describing whether light source is found.*/
	private boolean lightSourceFound = false;
	/**Used to keep a track of the maximum light reading encountered when making a 360 turn.*/
	private int maxLightReading = 0;
	/**Instantaneous light reading.*/
	private int currentLightReading = 0;
	/**The heading at which the maximum light reading occurred.*/
	private double angleOfMaxLightReading = 0;
	/**The distance to the light source.*/
	private int distanceToLightSource = 100;
	/**An array which holds the odometry information for the robot at a given point in time.*/
	private double[] pos = new double[3];
	/**This is the default path to follow while searching for the light source.*/
	private final double[][] pathToFollow = { { 0, 0 }, { 0, 30.48 },
			{ 0, 60.96 }, { 0, 91.44 }, { 0, 121.92 }, { 0, 121.92 },
			{ 30.48, 121.92 }, { 60.96, 121.92 }, { 91.44, 121.92 },
			{ 121.92, 121.92 }, { 121.92, 121.92 }, { 121.92, 91.44 },
			{ 121.92, 60.96 }, { 121.92, 30.48 }, { 121.92, 0 }, { 121.92, 0 },
			{ 91.44, 0 }, { 60.96, 0 }, { 30.48, 0 }, { 0, 0 } };

	
	/**This method is periodically called when the robot is rotating 360 degrees. It stores the 
	 * angle at which the maximum light reading occurs while the rotation is done.
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
	public FieldScanner(Odometer odo, UltrasonicSensor us, LightSensor ls) {
		this.us = us;
		this.ls = ls;
		this.odo = odo;
		this.nav = new Navigation(odo);
		ls.setFloodlight(false);
	}

	/**
	 * This method is called when the timer restarts (i.e. when the robot commences to do a 
	 * 360 degree rotation to locate the light source).
	 */
	public void reset() {
		maxLightReading = 0;
	}

	/**
	 * This method follows a particular path that is defined in an array. At each stop in the path
	 * this method calls another method to attempt to locate the light source. If the light source
	 * is located, the method will cause the robot to stop following the given path and travel 
	 * towards the light source.
	 */
	public void locateLightSourceAndGoToIt() {
		for (int i = 0; i < pathToFollow.length && !lightSourceFound; i++) {
			nav.travelTo(pathToFollow[i][0], pathToFollow[i][1]);
			RConsole.println("Travelling to coordinates: ("
					+ pathToFollow[i][0] + "," + pathToFollow[i][1] + ")");
			findLightSourceHeading();
			if (maxLightReading > MIN_LIGHT_INTENSITY ) {
				navigateTowardsLightSource();
			}
		}
	}

	/**
	 * 
	 */
	public boolean LightSourceFound(){
		if(maxLightReading>MIN_LIGHT_INTENSITY){
			return true;
		}
		
		return false;
	}
	
	public void turnToLightSourceHeading(){
		nav.turnTo(angleOfMaxLightReading);
	}
	
	public void locateLightSource(){
		reset();
		Timer timer = new Timer(10,this);
		timer.start();
		nav.turn360();
		timer.stop();
	}
	
	/**
	 * The goal of this method is to locate the light source. It accomplishes this by doing a 
	 * rotation of 360 degrees and turning towards the light source if it is found.
	 */
	public void findLightSourceHeading() {
		reset();
		Timer timer = new Timer(10, this);
		timer.start();
		nav.turn360();
		timer.stop();
		if(maxLightReading > MIN_LIGHT_INTENSITY){
			nav.turnTo(angleOfMaxLightReading);
		}
	}
	
	
	/**
	 * This method is called once a light source is located and causes the robot to travel towards it.
	 * While traveling towards the light source, it periodically attempts to correct the heading
	 * of the robot so that the final position of the robot is within 30 degrees of the light 
	 * source.
	 */
	public void navigateTowardsLightSource() {
		long start, end;//Used to keep track of when to apply the correction.
		distanceToLightSource = us.getDistance();
		start = System.currentTimeMillis();
		while (distanceToLightSource >= MIN_DIS_TO_LS) {
			nav.goStraight();
			distanceToLightSource = us.getDistance();
			RConsole.println("Distance to light source: "
					+ distanceToLightSource);
			end = System.currentTimeMillis();
			try {//The period for the correction is specified by UPDATE_HEADING_PERIOD.
				if (end - start > UPDATE_HEADING_PERIOD) {
					findLightSourceHeading();
					start = System.currentTimeMillis();
				} else {
					Thread.sleep(10);
				}
			} catch (InterruptedException e) {

			}
		}
		nav.stopGoingStraight();
		lightSourceFound = true;
	}
}
