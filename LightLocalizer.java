import lejos.nxt.LightSensor;
import lejos.nxt.Sound;

public class LightLocalizer {
	private static int X_INITIAL = 0;
	private static int X_FINAL = 2;
	private static int Y_INITIAL = 1;
	private static int Y_FINAL = 3;
	private Odometer odo;
	private TwoWheeledRobot robot;
	private LightSensor ls;
	private double ROTATION_SPEED = 20;
	public double CENTER_ROTATION = 10.5;
	public double BLACK_LINE = 42;
	
	public LightLocalizer(Odometer odo, LightSensor ls) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.ls = ls;
		
		// turn on the light
		ls.setFloodlight(true);
	}
	
	public void doLocalization() {
		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
		int count = 0;
		int line = 0;
		double[] position = new double[3];
		double[] lineCrossings = new double [4];
		Navigation navigation = Navigation.getNavigation(odo);
		double thetaX,thetaY,distX,distY,deltaTheta;
		
		//drive to the location listed in the tutorial
		navigation.travelTo(-1,-1);
		
		//odo.setPosition(new double [] {0.0, 0.0, 0.0 }, new boolean [] {true, true, true});
		while(line < 4){
			robot.setRotationSpeed(ROTATION_SPEED);
			if(ls.getLightValue() < BLACK_LINE){
				count++;
			}else{
				count = 0;
			}
			if(count == 2){
				odo.getPosition(position);
				lineCrossings[line] = position[2];
				line++;
				Sound.beep();
			}
		}
		robot.setRotationSpeed(0);
	
		// do trig to compute (0,0) and 0 degrees
		thetaX = lineCrossings[X_FINAL] - lineCrossings[X_INITIAL];
		thetaY = lineCrossings[Y_FINAL] - lineCrossings[Y_INITIAL];
		distX = -CENTER_ROTATION * Math.cos(Math.toRadians(thetaY/2)); //formula to calculate x coordinate
		distY = -CENTER_ROTATION * Math.cos(Math.toRadians(thetaX/2)); //formula to calculate y coordinate
		deltaTheta = 270+thetaY/2; //formula to calculate correct heading
		odo.setPosition(new double [] {distX, distY, deltaTheta}, new boolean [] {true, true, true});
				
		// when done travel to (0,0) and turn to 0 degrees
		navigation.travelTo(0, 0); 
		navigation.turnTo(0);
	}

}
