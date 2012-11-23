import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.RConsole;


public class LocTests {
	public static void main(String[] args) {
		int buttonChoice;
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
		
		do {
			LCD.drawString("Left - Run test", 0, 0);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT);
		
		usl.doLocalization();
		ll.doLocalization();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	}
}
