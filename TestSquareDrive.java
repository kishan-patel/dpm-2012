
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.RConsole;

public class TestSquareDrive {
	public static void main(String[] args) {
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
		

		
		nav.travelToUsingSearchAlgo(60,0);
		nav.travelToUsingSearchAlgo(60,60);
		nav.travelToUsingSearchAlgo(0, 60);
		nav.travelToUsingSearchAlgo(0, 0);
		//nav.traveToUsingSearchAlgo(0, 200);
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	}
}
