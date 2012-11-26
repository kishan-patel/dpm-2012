import lejos.util.Timer;
import lejos.util.TimerListener;

public class Odometer implements TimerListener {
	public static final int DEFAULT_PERIOD = 25;
	private TwoWheeledRobot robot;
	private Timer odometerTimer;
	// position data
	private static Object lock;
	private static double x;
	private static double y;
	private static double theta;
	private double [] oldDH, dDH;
	private static Coordinates coords = new Coordinates();
	
	public Odometer(TwoWheeledRobot robot, int period, boolean start) {
		// initialise variables
		this.robot = robot;
		odometerTimer = new Timer(period, this);
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		oldDH = new double [2];
		dDH = new double [2];
		lock = new Object();
		
		// start the odometer immediately, if necessary
		if (start)
			odometerTimer.start();
	}
	
	public Odometer(TwoWheeledRobot robot) {
		this(robot, DEFAULT_PERIOD, false);
	}
	
	public Odometer(TwoWheeledRobot robot, boolean start) {
		this(robot, DEFAULT_PERIOD, start);
	}
	
	public Odometer(TwoWheeledRobot robot, int period) {
		this(robot, period, false);
	}
	
	public void timedOut() {
		robot.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];
		
		// update the position in a critical region
		synchronized (lock) {
			theta += dDH[1];
			theta = fixDegAngle(theta);
			
			x += dDH[0] * Math.sin(Math.toRadians(theta));
			y += dDH[0] * Math.cos(Math.toRadians(theta));
		}
		
		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}
	
	// accessors
	public void getPosition(double [] pos) {
		synchronized (lock) {
			pos[0] = x;
			pos[1] = y;
			pos[2] = theta;
		}
	}
	
	public double getXPos(){
		synchronized (lock) {
			return x;
		}
	}
	
	public double getYPos(){
		synchronized (lock) {
			return y;
		}
	}
	
	public double getTheta(){
		synchronized (lock) {
			return theta;
		}
	}
	
	public TwoWheeledRobot getTwoWheeledRobot() {
		return robot;
	}
	
	
	// mutators
	public void setPosition(double [] pos, boolean [] update) {
		synchronized (lock) {
			if (update[0]) x = pos[0];
			if (update[1]) y = pos[1];
			if (update[2]) theta = pos[2];
		}
	}
	
	// static 'helper' methods
	public static double fixDegAngle(double angle) {		
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);
		
		return angle % 360.0;
	}
	
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);
		
		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}
	
	/**
	 * @return the coordinates of the robot
	 */
	public static Coordinates getCoordinates()
	{
		coords = new Coordinates();
		synchronized (lock) 
		{		
			coords.x = x;
			coords.y = y;
			coords.theta = theta;
		}
		return coords;
	}

	/**
	 * 
	 * @param coordsIn
	 * @param update an array of booleans representing which values are to be updated
	 */
	public static void setCoordinates(Coordinates coordsIn , boolean[] update) 
	{
		// ensure that the values don't change while the odometer is running
		synchronized (lock) 
		{
			if (update[0])
				x = coordsIn.x;
			if (update[1])
				y = coordsIn.y;
			if (update[2])
				theta = coordsIn.theta;
		}
	}
}
