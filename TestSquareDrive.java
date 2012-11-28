
import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.comm.RConsole;

public class TestSquareDrive {
	public static void main(String[] args) {
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
		
		//usl.doLocalization();
		
		USFilter usFilter = new USFilter();
		usFilter.start();
		
		LightFilter lsFilter = new LightFilter();
		lsFilter.start();
		
		/*try{Thread.sleep(1000);}catch (Exception e) {
			// TODO: handle exception
		}*/
		
		nav.travelToInXandY(60,0);
		//nav.turn360();
		//try{Thread.sleep(2000);}catch(InterruptedException e){}
		
		nav.travelToInXandY(60,60);
		//nav.turn360();
		//try{Thread.sleep(2000);}catch(InterruptedException e){}
		
		nav.travelToInXandY(0,60);
		//nav.turn360();
		//try{Thread.sleep(2000);}catch(InterruptedException e){}
		
		nav.travelToInXandY(0,0);
		
		//nav.turn360();
		//try{Thread.sleep(2000);}catch(InterruptedException e){}
		
		/*nav.turn360();
		
		try{Thread.sleep(1000);}catch(InterruptedException e){}
		nav.travelToInXandY(60,60);
		nav.turn360();
		nav.travelToInXandY(60,0);
		nav.turn360();
		nav.travelToInXandY(0, 0);*/

		
		//nav.turn360();
		//nav.traveToUsingSearchAlgo(0, 200);
	//	nav.travelToInXandY(60, 0);
		//nav.turn360();
		//nav.travelToInXandY(60,60);
		//nav.turn360();
		//nav.travelToInXandY(0, 60);
		//nav.turn360();
		//nav.travelToInXandY(0, 0);
		//nav.turn360();
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	}
}
