import java.io.DataInputStream;
import java.io.DataOutputStream;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.RConsole;
import bluetooth.BluetoothConnection;
import bluetooth.PlayerRole;
import bluetooth.StartCorner;
import bluetooth.Transmission;


public class MainMaster {
	//Variables used in establishing connection to the bluetooth server.
	private static BluetoothConnection conn;
	private static Transmission t;
	private static StartCorner corner;
	private static PlayerRole role;
	private static int dx,dy;
	
	//Variables used for communication with slave.
	private static NXTConnection connectionToSlave;
	private static DataInputStream dis;
	private static DataOutputStream dos;
	private static int OPEN = -1;
	private static int GROUND_HEIGHT = 0;
	
	//Variables used for by the attacker/defender.
	private static USSensor usSensor;
	private static TwoWheeledRobot patBot;
	private static Odometer odo;
	private static LCDInfo lcd;
	private static Navigation nav;
	private static FieldScanner fieldScanner;
	private static SearchAlgorithm searchAlgorithm;
	private static USLocalizer usl;
	private static LSLocalizer ls;
	
	public static void main(String[] args){
		//connectToBTServer();
		//connectToSlave();
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
			LightSensor ls = SensorAndMotorInfo.getRightLightSensor();
			while(true){
				RConsole.println("LV: "+ls.getLightValue());
				try{Thread.sleep(100);}catch(InterruptedException e){}
				
			}
			
		}else if (buttonChoice == Button.ID_RIGHT){
			//Attacker code
			findAndGoToBeacon();
			/*try{
				dos.writeInt(OPEN);
				dos.flush();
				LCD.drawString("Sent open instruc.",0,2);
				while(dis.available()<=0){
					Thread.sleep(10);
				}
				dis.readBoolean();
				LCD.drawString("Got open conf.", 0, 2);
				dos.writeInt(GROUND_HEIGHT);
				dos.flush();
				while(dis.available()<=0){
					Thread.sleep(10);
				}
				dis.readBoolean();
				nav.traveToUsingSearchAlgo(dx, dy);
			}catch(IOException e){
				
			}catch(InterruptedException e){
				
			}*/
			
			
		}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	}
	
	public static void connectToBTServer(){
		conn = new BluetoothConnection();
		t = conn.getTransmission();
		
		if (t == null) {
			LCD.drawString("Failed to read transmission", 0, 5);
		} else {
			corner = t.startingCorner;
			role = t.role;
			// attacker will drop the flag off here
			dx = t.dx;	//destination pos x
			dy = t.dy;	//destination pos y
		}
	}
	
	public static void connectToSlave(){
		connectionToSlave = Bluetooth.connect("ShootingBrick",NXTConnection.PACKET);
		try {
			dis = connectionToSlave.openDataInputStream();
			dos = connectionToSlave.openDataOutputStream();
		} catch (Exception e) {
			connectToSlave();
		}
	}
	
	public static void findAndGoToBeacon(){
		//Variables used for attacking/defending.
		USSensor usSensor = SensorAndMotorInfo.getUsSensor();
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		Odometer odo = new Odometer(patBot,true);
		LCDInfo lcd = new LCDInfo(odo);
		Navigation nav = Navigation.getNavigation(odo);
		FieldScanner fieldScanner = FieldScanner.getFieldScanner(odo);
		SearchAlgorithm searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();
		USLocalizer usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);
		LightLocalizer ls = new LightLocalizer(odo,SensorAndMotorInfo.RIGHT_LIGHT_SENSOR);
		boolean beaconFound = false;
		double[] nextSearchLocation;
		boolean isBeaconDetectedByUS = false;
		int distToBeacon;
		double[] position = new double[3];
		/*Process of steps that are executed in order to find the beacon and travel to it.*/
		//usl.doLocalization();
		//try{Thread.sleep(5000);}catch(InterruptedException e){}
		ls.doLocalization();
		
		
		/*while(!beaconFound){
			fieldScanner.locateBeacon();
			
			if(!fieldScanner.beaconLocated()){
				//The beacon has not yet been located. Thus, we go to the next position
				//in our search algorithm.
				RConsole.println("Beacon not located");
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
				RConsole.println("Distance to beacon: "+distToBeacon);
				if(distToBeacon<30.48){
					RConsole.println("Distance to beacon is within 1 tile");
					beaconFound = true;
					break;
				}else{
					odo.getPosition(position);
					nextSearchLocation = searchAlgorithm.getNextLocCloserToBeacon(position[0], position[1], position[2], distToBeacon);
					RConsole.println("Next x: "+nextSearchLocation[0]);
					RConsole.println("Next y: "+nextSearchLocation[1]);
					nav.traveToUsingSearchAlgo(nextSearchLocation[0], nextSearchLocation[1]);
					
				} 
				
				nav.navigateTowardsLightSource(20);
				nav.turnTo(odo.getTheta()-15.0);
				nav.navigateTowardsLightSource(5);
				beaconFound = true;
				break;
			} 
		}*/
	}
}
