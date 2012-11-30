import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.comm.RConsole;



public class BeaconFindTest {
	public static void main(String[] args) {
		RConsole.openUSB(6000);
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
		
		/*for(int i=0;i<10;i++){
			fieldScanner.locateBeacon();
			fieldScanner.turnToBeacon();
			try{Thread.sleep(3000);}catch(InterruptedException e){}
		}*/
		
		while(true){
			RConsole.println("LV: "+lf.getBeaconLight());
			RConsole.println("dist: "+usf.getUS());
			try{Thread.sleep(100);}catch(InterruptedException e){}
		}
	

	}
}
