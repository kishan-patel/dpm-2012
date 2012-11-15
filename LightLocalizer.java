import lejos.nxt.LightSensor;
import lejos.nxt.Sound;

public class LightLocalizer {

	private Odometer odo;
	private TwoWheeledRobot robot;
	private LightSensor ls;
	private double ROTATION_SPEED = 20;
	public double CENTER_ROTATION = 10.5;
	public double BLACK_LINE = 42;
	
	private double thetaX, thetaY, thetaZ, distX, distY, deltaTheta;
	private int stage;
	
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
		
		
		//drive to the location listed in the tutorial
		navigation.travelTo(-5,-5);
		
		robot.setRotationSpeed(-ROTATION_SPEED);
		
		//when first line is crossed, grab heading value from odometer
		while(ls.getLightValue() < BLACK_LINE && stage==0){
			thetaY = odo.getTheta();
		} stage++;
		
		//when second line crossed, grab heading value from odometer
		while(ls.getLightValue() < BLACK_LINE && stage==1){
			thetaX = odo.getTheta();
		} stage++;
		
		//when third line crossed, update thetaY value
		while(ls.getLightValue() < BLACK_LINE && stage==2){
			thetaZ = odo.getTheta();
		} 
		thetaY = thetaZ - thetaY;
		stage++;
		
		//when fourth line crossed, update thetaX value
		while(ls.getLightValue() < BLACK_LINE && stage==3){
			thetaZ = odo.getTheta();
		}
		thetaX = thetaZ - thetaX;
		
		//calculate position
		distX = -CENTER_ROTATION*Math.sin(Math.toRadians(thetaY/2));
		distY = -CENTER_ROTATION*Math.sin(Math.toRadians(thetaX/2));
		
		deltaTheta = 270+thetaY/2; //formula to calculate correct heading
		odo.setPosition(new double [] {distX, distY, deltaTheta}, new boolean [] {true, true, true});
		
		//odo.setPosition(new double [] {0.0, 0.0, 0.0 }, new boolean [] {true, true, true});
		/*while(line < 4){
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
		*/
		robot.setRotationSpeed(0);
	
		
				
		// when done travel to (0,0) and turn to 0 degrees
		navigation.travelTo(0, 0); 
		navigation.turnTo(0);
	}

}
