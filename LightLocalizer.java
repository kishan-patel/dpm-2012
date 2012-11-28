import lejos.nxt.LightSensor;
import lejos.nxt.Sound;
import lejos.nxt.comm.RConsole;

public class LightLocalizer {

	private Odometer odo;
	private TwoWheeledRobot robot;
	private LightSensor ls;
	private double ROTATION_SPEED = 20;
	public double CENTER_ROTATION = 14.5;
	public double BLACK_LINE = 48;
	
	//private double thetaX, thetaY, thetaZ, distX, distY, deltaTheta;
	
	private double thetaX1,thetaX2,thetaY1,thetaY2,deltaThetaX,deltaThetaY,distX,distY,deltaTheta;
	
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
		Navigation nav = Navigation.getNavigation(odo);
		int lightValue;
		int lineDetections=0;
		nav.travelTo(4,0);
		nav.turnTo(odo.getTheta()+20);
		//drive to the location listed in the tutorial
		robot.setRotationSpeed(ROTATION_SPEED);
		robot.setRotationSpeed(ROTATION_SPEED);
		//when first line is crossed, grab heading value from odometer
		while((lightValue=LightFilter.getLeftLight()) > Constants.LEFT_LIGHT_THRESHOLD){
			RConsole.println("LV 1: "+lightValue);
		}
		RConsole.println(""+lightValue);
		thetaX1 = odo.getTheta();
		Sound.beep();
		RConsole.println("firstline");
		lineDetections = 0;
		try{Thread.sleep(500);}catch(InterruptedException e){}

		//when second line crossed, grab heading value from odometer
		while((lightValue=LightFilter.getLeftLight())  > Constants.LEFT_LIGHT_THRESHOLD){
			RConsole.println("LV 2: "+lightValue);
		}	
		RConsole.println(""+lightValue);
		thetaY1 = odo.getTheta();
		Sound.beep();
		RConsole.println("secondline");
		lineDetections = 0;
		try{Thread.sleep(500);}catch(InterruptedException e){}

		
		//when third line crossed, update thetaY value
		while((lightValue=LightFilter.getLeftLight())  > Constants.LEFT_LIGHT_THRESHOLD){
			RConsole.println("LV 3: "+lightValue);
		}
		RConsole.println(""+lightValue);
		thetaX2 = odo.getTheta();
		Sound.beep();
		RConsole.println("thirdline");
		lineDetections = 0;
		try{Thread.sleep(500);}catch(InterruptedException e){}
		
		//when fourth line crossed, update thetaX value
		while((lightValue=LightFilter.getLeftLight())  > Constants.LEFT_LIGHT_THRESHOLD){
			RConsole.println("LV 4: "+lightValue);
		}
		
		RConsole.println(""+lightValue);
		thetaY2 = odo.getTheta();
		Sound.beep();
		RConsole.println("fourthline");
		
		//calculate position
		// do trig to compute (0,0) and 0 degrees
		deltaThetaX = thetaX2 - thetaX1;
		deltaThetaY = thetaY2 - thetaY1;
		distY = CENTER_ROTATION * Math.cos(Math.toRadians(deltaThetaY/2)); //formula to calculate x coordinate
		distX = -CENTER_ROTATION * Math.cos(Math.toRadians(deltaThetaX/2)); //formula to calculate y coordinate
		deltaTheta = 205+deltaThetaY/2; //formula to calculate correct heading
		odo.setPosition(new double [] {distX-3, distY, deltaTheta}, new boolean [] {true, true, true});
		//nav.travelTo(0, 0);
		//odo.setPosition(new double [] {0.0, 0.0, 0.0 }, new boolean [] {true, true, true});
		nav.turnTo(0);
		robot.setRotationSpeed(0);
		robot.setRotationSpeed(0);
		
				
		// when done travel to (0,0) 
		
		//try{Thread.sleep(15000);}catch(InterruptedException e) {}
	}
}
