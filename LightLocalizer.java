import lejos.nxt.LightSensor;
import lejos.nxt.comm.RConsole;

public class LightLocalizer {

	private Odometer odo;
	private TwoWheeledRobot robot;
	private LightSensor ls;
	private double ROTATION_SPEED = 20;
	public double CENTER_ROTATION = 17.5;
	public double BLACK_LINE = 40;
	
	private double thetaX, thetaY, thetaZ, distX, distY, deltaTheta;
	
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
		int lightValue;
		
		//drive to the location listed in the tutorial
		navigation.turnTo(90);
		
		try{Thread.sleep(2000);}catch(InterruptedException e) {}
		
		robot.setRotationSpeed(-ROTATION_SPEED);
		robot.setRotationSpeed(-ROTATION_SPEED);
		
		//when first line is crossed, grab heading value from odometer
		while((lightValue=ls.getLightValue()) > BLACK_LINE){
			RConsole.println("LV 1: "+lightValue);
		}
		
		thetaY = odo.getTheta();
		try{Thread.sleep(1000);}catch(InterruptedException e) {}
		RConsole.println("firstline");
		
		//when second line crossed, grab heading value from odometer
		while((lightValue=ls.getLightValue()) > BLACK_LINE ){
			RConsole.println("LV 2: "+lightValue);
		}	
		
		thetaX = odo.getTheta();
		try{Thread.sleep(1000);}catch(InterruptedException e) {}
		RConsole.println("secondline");
		
		//when third line crossed, update thetaY value
		while((lightValue=ls.getLightValue()) > BLACK_LINE){
			RConsole.println("LV 3: "+lightValue);
		}
		thetaZ = odo.getTheta();
		try{Thread.sleep(1000);}catch(InterruptedException e) {}
		thetaY = thetaZ - thetaY;
		RConsole.println("thirdline");
		
		//when fourth line crossed, update thetaX value
		while((lightValue=ls.getLightValue()) > BLACK_LINE){
			RConsole.println("LV 4: "+lightValue);
		}
		
		thetaZ = odo.getTheta();
		try{Thread.sleep(1000);}catch(InterruptedException e) {}
		thetaX = thetaZ - thetaX;
		RConsole.println("fourthline");
		RConsole.println("thetaY: "+thetaY);
		RConsole.println("thetaX: "+thetaX);
		//calculate position
		distX = -CENTER_ROTATION*Math.sin(Math.toRadians(thetaY/2));
		distY = -CENTER_ROTATION*Math.sin(Math.toRadians(thetaX/2));
		
		RConsole.println("distY: "+distY);
		RConsole.println("distX: "+distX);
		
		RConsole.println("found distances");
		deltaTheta = 270+thetaY/2; //formula to calculate correct heading
		
		RConsole.println("deltaTheta: "+ deltaTheta);
		
		odo.setPosition(new double [] {distX, distY, deltaTheta}, new boolean [] {true, true, true});
		
		RConsole.println("set new position");
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
		robot.setRotationSpeed(0);
		
				
		// when done travel to (0,0)
		navigation.travelTo(0, 0); 
		
		try{Thread.sleep(15000);}catch(InterruptedException e) {}
	}

}
