import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.RConsole;



public class TestAvoid {
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
		Navigation  nav = Navigation.getNavigation(odo);
		FieldScanner  fieldScanner = FieldScanner.getFieldScanner(odo);;
		SearchAlgorithm searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();;
		USLocalizer usl = usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);;
		LightLocalizer ll = new LightLocalizer(odo, SensorAndMotorInfo.LS_LOCALIZATION_SENSOR);
		LCD.clear();
		
		LightFilter lf = new LightFilter();
		lf.start();
		
		USFilter usf = new USFilter();
		usf.start();
		
		 double dxCoordinate = 15.24;
		 double dyCoordinate = 213.6;
		 nav.travelToInXandY(dxCoordinate, dyCoordinate);
	}
}
