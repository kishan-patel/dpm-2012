import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;
import lejos.util.Timer;


public class Main {

	public static void main(String[] args){
		RConsole.openBluetooth(5000);
		int buttonChoice;
		LCD.clear();
		do{
			LCD.drawString("Left - Do Us. Loca." , 0, 0);
			buttonChoice = Button.waitForAnyPress();
		}while(buttonChoice!=Button.ID_RIGHT&&buttonChoice!=Button.ID_LEFT);

		
		if(buttonChoice == Button.ID_LEFT){
			USSensor us = new USSensor(SensorPort.S1);
			LightSensor ls = new LightSensor(SensorPort.S3);
			TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
			Odometer odo = new Odometer(patBot, true);
			LCDInfo lcd = new LCDInfo(odo);
			USLocalizer usl = new USLocalizer(odo, us,
					USLocalizer.LocalizationType.FALLING_EDGE);
			//LightFinder lf = new LightFinder(odo, us, ls);
			usl.doLocalization();
		Timer ab;
		}else if (buttonChoice == Button.ID_RIGHT){
			USSensor usSensor = SensorAndMotorInfo.getUsSensor();
			/*while(true){
				RConsole.println("dist: "+usSensor.getDistance());
				try{
					Thread.sleep(10);
				}catch(InterruptedException e){
					
				}
			}*/
			//LightSensor leftLS = SensorAndMotorInfo.getLeftLightSensor();
			//LightSensor rightLS = SensorAndMotorInfo.getRightLightSensor();
			TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
			RConsole.println("Got two wheeled robot");
			Odometer odo = new Odometer(patBot,true);
			RConsole.println("Got odo");
			LCDInfo lcd = new LCDInfo(odo);
			RConsole.println("Got lcd");
			USLocalizer usl = new USLocalizer(odo, usSensor,
					USLocalizer.LocalizationType.FALLING_EDGE);
			RConsole.println("Created usl object");
			usl.doLocalization();
			Navigation nav = Navigation.getNavigation(odo);
			RConsole.println("Got navigation object");
			//nav.moveTo(0,2);
			//nav.travelTo(-15.24,-15.24);
			//nav.turnTo(0.0);
			nav.traveToUsingSearchAlgo(60,60);
			nav.stopGoingStraight();
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	
		
	}
}
