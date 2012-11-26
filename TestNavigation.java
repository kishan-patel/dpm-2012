import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.RConsole;


public class TestNavigation {

	public static void main(String[] args) {
		int buttonChoice;

		do {
			LCD.drawString("Left - Run test", 0, 0);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT);

		// Variables used for by the attacker/defender.
		USSensor usSensor = SensorAndMotorInfo.US_SENSOR;
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		Odometer odo = new Odometer(patBot, true);
		LCDInfo lcd = new LCDInfo(odo);
		Navigation nav = Navigation.getNavigation(odo);
		USLocalizer usl = new USLocalizer(odo, usSensor,
				USLocalizer.LocalizationType.FALLING_EDGE);
		LightLocalizer ll = new LightLocalizer(odo,
				SensorAndMotorInfo.LS_LOCALIZATION_SENSOR);
		//usl.doLocalization();
	//	ll.doLocalization();
		USFilter usFilter = new USFilter();
		usFilter.start();
		
		LightFilter lsFilter = new LightFilter();
		lsFilter.start();
		
		nav.travelToInXandY(15.24,15.24);
		nav.turn360();
		nav.travelToInXandY(45.72, 15.24);
		nav.turn360();
		nav.travelToInXandY(76.2,15.24);
		nav.turn360();
		nav.travelToInXandY(76.2,45.72);
		nav.turn360();
		nav.travelToInXandY(45.72,45.72);
		nav.turn360();
		nav.travelToInXandY(15.24,45.72);
		nav.turn360();
		nav.travelToInXandY(15.24,76.2);
		nav.turn360();
		nav.travelToInXandY(76.2,76.2);
		//nav.travelToInXandY(0, 0);
		
		
		
	/*	nav.travelToInXandY(60,0);
		nav.travelToInXandY(60, 60);
		nav.travelToInXandY(0,60);
		nav.travelToInXandY(0, 0);*/

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		RConsole.close();
		System.exit(0);
	}
}
