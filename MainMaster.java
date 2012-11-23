import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

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
	private static int dx=10,dy=10;
	
	//Variables used for communication with slave.
	private static NXTConnection connectionToSlave;
	private static DataInputStream dis;
	private static DataOutputStream dos;
	public static final int CLOSE_CLAW = 0;
	public static final int OPEN_CLAW = 1;
	public static final int LOWER_CLAW_TO_FLOOR = 2;
	public static final int MOVE_CLAW_UP = 3;
	
	//Variables used for by the attacker/defender.
	private static USSensor usSensor;
	private static TwoWheeledRobot patBot;
	private static Odometer odo;
	private static LCDInfo lcd;
	private static Navigation nav;
	private static FieldScanner fieldScanner;
	private static SearchAlgorithm searchAlgorithm;
	private static USLocalizer usl;
	private static LightLocalizer ll;
	
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
		}else if (buttonChoice == Button.ID_RIGHT){
			//Attacker code
			findAndGoToBeacon();
			pickupBeacon();
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
		// Variables used for attacking/defending.
		usSensor = SensorAndMotorInfo.US_SENSOR;
		patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		odo = new Odometer(patBot, true);
		lcd = new LCDInfo(odo);
		nav = Navigation.getNavigation(odo);
		fieldScanner = FieldScanner.getFieldScanner(odo);
		searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();
		usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);
		ll = new LightLocalizer(odo, SensorAndMotorInfo.LS_LOCALIZATION_SENSOR);
		boolean beaconFound = false;
		double[] nextSearchLocation;
		
		usl.doLocalization();
		try{Thread.sleep(5000);}catch(InterruptedException e){}
		ll.doLocalization();

		while (!beaconFound) {
			fieldScanner.locateBeacon();

			if (!fieldScanner.beaconLocated()) {
				// The beacon has not yet been located. Thus, we go to the next
				// position
				// in our search algorithm.
				RConsole.println("Beacon not located");
				nextSearchLocation = searchAlgorithm.getNextSearchLocation();
				if (nextSearchLocation == null) {
					beaconFound = true;
					break;
				} else {
					nav.traveToUsingSearchAlgo(nextSearchLocation[0],
							nextSearchLocation[1]);
				}
			} else {
				fieldScanner.turnToBeacon();
				RConsole.println("Beacon located. turned to it. Headed towards it.");
				nav.navigateTowardsLightSource(30);
				fieldScanner.locateBeacon();
				fieldScanner.turnToBeacon();
				RConsole.println("Turing 10 degrees CCW");
				nav.turnTo(odo.getTheta() - 180);
				RConsole.println("Moving 10 cm forward");
				nav.goStraight(30);
				beaconFound = true;
				break;
			}
		}
	}
	
	public static void pickupBeacon() {
		try {
			dos.writeInt(LOWER_CLAW_TO_FLOOR);
			dos.flush();
			while (dis.available() <= 0) {
				Thread.sleep(10);
			}
			dis.readBoolean();
			Thread.sleep(2000);

			dos.writeInt(CLOSE_CLAW);
			dos.flush();
			while (dis.available() <= 0) {
				Thread.sleep(10);
			}
			dis.readBoolean();
			Thread.sleep(4000);

			dos.writeInt(MOVE_CLAW_UP);
			dos.flush();
			while (dis.available() <= 0) {
				Thread.sleep(10);
			}
			dis.readBoolean();
			Thread.sleep(4000);

			while (true) {
				Thread.sleep(30);
			}

		} catch (InterruptedException e) {
		} catch (IOException e) {
		} catch (Exception e) {
		} finally {
		}
	}
}
