import lejos.nxt.LightSensor;
import lejos.nxt.comm.RConsole;
	
	
	public class LSLocalizer {
		
		/**The light sensor is used to find the angles necessary for localization*/
		private LightSensor lsright;
		
		/**An instance of an odometer class. It's used to get the robot's odometry information.*/
		private Odometer odo;
		
		/**Used to set the speeds of the robot.*/
		private TwoWheeledRobot robot;
		
		/**Used to control the forward motion and rotation of the robot.*/
		private Navigation nav;
		
		/**Distance from light sensors to the center of rotation**/
		private final double DCENTER = 10.0;
		
		/**Speed of robot's rotation**/
		private final double ROTATION_SPEED = -10.0;
		
		/**Low light value signifying a black line**/
		private final double LOW_LIGHT = 40;
		
		
		/**thetaY and thetaX represent the angles found in the
		mathematical derivation for light sensor localization **/
		private double thetaZ;
		private double thetaY;
		private double thetaX;
		private double heading;

		
		/**distances from tile center**/
		private double x;
		private double y;
		
		/** stage of localization**/
		private double stage = 0;
	
		//Constructor
		public LSLocalizer(Odometer odo, LightSensor ls1) {
		this.lsright = ls1;
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.nav = Navigation.getNavigation(odo);
		
		}
		
		//Localization method
		public void doLocalization(){
			
			RConsole.println("light");
			
			//travel to a point where localization can begin
			nav.travelTo(0, 0);
			
			RConsole.println("Got navigation");
			
			//begin turning
			robot.setRotationSpeed(ROTATION_SPEED);
			
			//when first line is crossed, grab heading value from odometer
			while(lsright.getLightValue() < LOW_LIGHT && stage==0){
				thetaY = odo.getTheta();
			} stage++;
			
			//when second line crossed, grab heading value from odometer
			while(lsright.getLightValue() < LOW_LIGHT && stage==1){
				thetaX = odo.getTheta();
			} stage++;
			
			//when third line crossed, update thetaY value
			while(lsright.getLightValue() < LOW_LIGHT && stage==2){
				thetaZ = odo.getTheta();
			} 
			thetaY = thetaZ - thetaY;
			stage++;
			
			//when fourth line crossed, update thetaX value
			while(lsright.getLightValue() < LOW_LIGHT && stage==3){
				thetaZ = odo.getTheta();
			}
			thetaX = thetaZ - thetaX;
			
			//calculate position
			x = - Math.sin(thetaY/2);
			y = - Math.sin(thetaX/2);
			
			//calculate heading
			heading = -thetaX/2;
			
			
			
			//set odometry values to those calculated
			odo.setPosition(new double [] {x, y, heading}, new boolean [] {true, true, true});
			
			nav.turnTo(0.0);
			
		}
		
		
		

}
