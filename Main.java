import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.comm.RConsole;
import lejos.util.Timer;


public class Main {

	public static void main(String[] args){
		RConsole.openBluetooth(5000);
		int buttonChoice;
		LCD.clear();
		do{
			LCD.drawString("Left - Defend" , 0, 0);
			LCD.drawString("Right - Attack",0,1);
			buttonChoice = Button.waitForAnyPress();
		}while(buttonChoice!=Button.ID_RIGHT&&buttonChoice!=Button.ID_LEFT);

		
		if(buttonChoice == Button.ID_LEFT){
			//Defender code
			USSensor usSensor = SensorAndMotorInfo.getUsSensor();
			TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
			Odometer odo = new Odometer(patBot,true);
			LCDInfo lcd = new LCDInfo(odo);
			Navigation nav = Navigation.getNavigation(odo);
			FieldScanner fieldScanner = FieldScanner.getFieldScanner(odo);
			SearchAlgorithm searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();
			USLocalizer usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);
			/*while(true){
				RConsole.println("Light reading: "+ls.getLightValue());
				try{
					Thread.sleep(100);
				}catch(InterruptedException e){
					
				}
			}*/
			usl.doLocalization();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			try{Thread.sleep(10000);}catch(InterruptedException e){}
			nav.turn360();
			nav.traveToUsingSearchAlgo(60.0, 60.0);
			/*fieldScanner.locateBeacon();
			fieldScanner.turnToBeacon();*/
		}else if (buttonChoice == Button.ID_RIGHT){
			//Attacker code
			//bluet
			USSensor usSensor = SensorAndMotorInfo.getUsSensor();
			TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
			Odometer odo = new Odometer(patBot,true);
			LCDInfo lcd = new LCDInfo(odo);
			Navigation nav = Navigation.getNavigation(odo);
			FieldScanner fieldScanner = FieldScanner.getFieldScanner(odo);
			SearchAlgorithm searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();
			USLocalizer usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);
			
			boolean beaconFound = false;
			double[] nextSearchLocation;
			boolean isBeaconDetectedByUS = false;
			int distToBeacon;
			double[] position = new double[3];
			/*Process of steps that are executed in order to find the beacon and travel to it.*/
			usl.doLocalization();
			while(!beaconFound){
				fieldScanner.locateBeacon();
				
				if(!fieldScanner.beaconLocated()){
					//The beacon has not yet been located. Thus, we go to the next position
					//in our search algorithm.
					nextSearchLocation = searchAlgorithm.getNextSearchLocation();
					if(nextSearchLocation == null){
						beaconFound = true;
						break;
					}else{
						nav.traveToUsingSearchAlgo(nextSearchLocation[0], nextSearchLocation[1]);
					}
				}else{
					//The beacon has been located so we use the search algorithm to get the next point
					//that will move the robot closer to the beacon.
					isBeaconDetectedByUS = fieldScanner.isBeaconDetectedByUS();
					if(isBeaconDetectedByUS){
						distToBeacon = fieldScanner.getDistanceToBeacon();
					}else{
						distToBeacon = 255;
					}
					
					if(distToBeacon<30.48){
						beaconFound = true;
						break;
					}else{
						odo.getPosition(position);
						nextSearchLocation = searchAlgorithm.getNextLocCloserToBeacon(position[0], position[1], position[2], distToBeacon);
						nav.traveToUsingSearchAlgo(nextSearchLocation[0], nextSearchLocation[1]);
					}
				}
			}
			
			//sen-data
			
			//move-closer 
			
			//
			
		
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	
		
	}
}
