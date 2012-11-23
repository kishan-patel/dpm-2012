import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.RConsole;



public class AvoidTest {
	public static void main(String[] args) {
		
		(new Thread() {public void run(){
			int choice = Button.waitForAnyPress();
			while(choice != Button.ID_ESCAPE){
				choice = Button.waitForAnyPress();
			}
			}
		}).start();
		RConsole.openBluetooth(5000);
		int buttonChoice;
		do {
			LCD.drawString("Left - Run test", 0, 0);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT);
		
		//Variables used for by the attacker/defender.
		USSensor  usSensor = SensorAndMotorInfo.US_SENSOR;;
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);;
		Odometer odo = new Odometer(patBot,true);;
		LCDInfo lcd = new LCDInfo(odo);
		OdoCorrection odoCorrection = new OdoCorrection(odo);;
		Navigation  nav = Navigation.getNavigation(odo);
		FieldScanner  fieldScanner = FieldScanner.getFieldScanner(odo);;
		SearchAlgorithm searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();;
		USLocalizer usl = usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);;
		LightLocalizer ll = new LightLocalizer(odo, SensorAndMotorInfo.LS_LOCALIZATION_SENSOR);
		final int FWD_SPEED = 10;
		final int distanceToStopAt = 31;
						
		int distanceToLightSource = usSensor.getDistance();
		int noOfObjectDetections=0;
		
		while(distanceToLightSource >= distanceToStopAt||noOfObjectDetections<=5){
			
			distanceToLightSource = usSensor.getDistance();
			
			patBot.setRotationSpeed(0.0);
			patBot.setForwardSpeed(FWD_SPEED);
			
			if( distanceToLightSource <= distanceToStopAt ){
				noOfObjectDetections++;
			}else{
				noOfObjectDetections=0;
			}
			
		}
		nav.avoidObstacle();
	}
}
