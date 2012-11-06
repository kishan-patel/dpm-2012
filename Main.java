import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Timer;
import lejos.util.TimerListener;


public class Main {

	public static void main(String[] args){
		int buttonChoice;
		LCD.clear();
		do{
			LCD.drawString("Left - Do Us. Loca." , 0, 0);
			buttonChoice = Button.waitForAnyPress();
		}while(buttonChoice!=Button.ID_RIGHT&&buttonChoice!=Button.ID_LEFT);

		
		if(buttonChoice == Button.ID_LEFT){
			UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
			LightSensor ls = new LightSensor(SensorPort.S3);
			TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
			Odometer odo = new Odometer(patBot, true);
			LCDInfo lcd = new LCDInfo(odo);
			USLocalizer usl = new USLocalizer(odo, us,
					USLocalizer.LocalizationType.FALLING_EDGE);
			//LightFinder lf = new LightFinder(odo, us, ls);
			usl.doLocalization();
		Timer ab;
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
	}
}
