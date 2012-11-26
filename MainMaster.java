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
	public static PlayerRole role;
	public static int dx=4,dy=4;
	public static int ax=1, ay=1;
	public static double dxCoordinate = (dx*30.48)-(30.48/2);
	public static double dyCoordinate = (dy*30.48)-(30.48/2);
	
	//Variables used for communication with slave.
	private static NXTConnection connectionToSlave;
	private static DataInputStream dis;
	private static DataOutputStream dos;
	public static final int CLOSE_CLAW = 0;
	public static final int OPEN_CLAW = 1;
	public static final int LOWER_CLAW_TO_FLOOR = 2;
	public static final int MOVE_CLAW_UP = 3;
	
	//Variables used for by the attacker/defender.
	private static USSensor usSensor = SensorAndMotorInfo.US_SENSOR;;
	private static TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
	private static Odometer odo = new Odometer(patBot, true);
	private static LCDInfo lcd;
	private static Navigation nav = Navigation.getNavigation(odo);
	private static FieldScanner fieldScanner = FieldScanner.getFieldScanner(odo);
	private static SearchAlgorithm 	searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();;
	private static USLocalizer usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);
	private static LightLocalizer ll = new LightLocalizer(odo, SensorAndMotorInfo.LS_LOCALIZATION_SENSOR);

	public static void main(String[] args){
		//connectToBTServer();
		connectToSlave();
		RConsole.openBluetooth(5000);
		LCD.clear();
		
		int buttonChoice;
		do{
			LCD.drawString("Left - Defend" , 0, 0);
			LCD.drawString("Right - Attack",0,1);
			buttonChoice = Button.waitForAnyPress();
		}while(buttonChoice!=Button.ID_RIGHT&&buttonChoice!=Button.ID_LEFT);

		if(buttonChoice == Button.ID_LEFT){
			lcd = new LCDInfo(odo);
			
			//Perform localization prior to going to the beacon.
			usl.doLocalization();
			try{Thread.sleep(2000);}catch(InterruptedException e){}
			ll.doLocalization();
			searchAlgorithm.setDefenderLocation(dx, dy);
			
			//Go to the beacon and stop at the optimal position.
			goToBeacon();
			pickupBeacon();
			nav.carryingBeacon = true;
			hideBeacon();
			dropBeacon();
			nav.travelToInXandY(Math.abs(odo.getXPos()-10), Math.abs(odo.getYPos()-10));
		}else if (buttonChoice == Button.ID_RIGHT){
			//Attacker code
			lcd = new LCDInfo(odo);
			
			//Perform localization prior to going to the beacon.
			usl.doLocalization();
			try{Thread.sleep(2000);}catch(InterruptedException e){}
			ll.doLocalization();
			
			//Pickup the beacon and drop it at the optimal location.
			findAndGoToBeacon();
			pickupBeacon();
			nav.travelToInXandY(ax, ay);
			dropBeacon();
			nav.travelToInXandY(odo.getXPos()-10, odo.getYPos()-10);
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
			//Roles and starting corner.
			corner = t.startingCorner;
			role = t.role;
			
			//Defender will pick the flag from here.
			dx = t.fx;
			dy = t.fy;
			
			// attacker will drop the flag off here.
			ax = t.dx;	
			ay = t.dy;	
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
	
	public static void goToBeacon(){
		boolean beaconFound = false;
		int maxLight;
		double[] searchLoc;
		
		while(!beaconFound){
			searchLoc = searchAlgorithm.getNextDefenderSearchLocation();
			if(searchLoc == null){
				beaconFound = true;
				break;
			}
			nav.travelToInXandY(searchLoc[0], searchLoc[1]);
			fieldScanner.locateBeacon();
			maxLight = fieldScanner.getMaxLightReading();
			if(maxLight > 33){
				goInBestPosition();
				beaconFound = true;
				break;
			}
		}
	}
	
	public static void hideBeacon(){
		nav.travelToInXandY(0,0);
	}
	
	public static void findAndGoToBeacon(){
		// Variables used for attacking/defending.
		boolean beaconFound = false;
		double[] nextSearchLocation;

		while (!beaconFound) {
			fieldScanner.locateBeacon();

			if (!fieldScanner.beaconLocated()) {
				// The beacon has not yet been located. Thus, we go to the next
				// position
				// in our search algorithm.
				nextSearchLocation = searchAlgorithm.getNextAttackerSearchLocation();
				if (nextSearchLocation == null) {
					beaconFound = true;
					break;
				} else {
					nav.travelToInXandY(nextSearchLocation[0],
							nextSearchLocation[1]);
				}
			} else {
				fieldScanner.turnToBeacon();
				nav.navigateTowardsLightSource(30);
				goInBestPosition();
				beaconFound = true;
				break;
				
			}
		}
	}
	
	public static void goInBestPosition(){
		fieldScanner.locateBeacon();
		fieldScanner.turnToBeacon();
		nav.turnTo(odo.getTheta() - 180);
		nav.goStraight(28);
		nav.turnTo(odo.getTheta() - 15);
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
			Thread.sleep(2000);

			dos.writeInt(MOVE_CLAW_UP);
			dos.flush();
			while (dis.available() <= 0) {
				Thread.sleep(10);
			}
			dis.readBoolean();
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		} catch (IOException e) {
		} catch (Exception e) {
		} finally {
		}
	}
	
	public static void dropBeacon(){
		try {
			dos.writeInt(LOWER_CLAW_TO_FLOOR);
			dos.flush();
			while (dis.available() <= 0) {
				Thread.sleep(10);
			}
			dis.readBoolean();
			Thread.sleep(2000);

			dos.writeInt(OPEN_CLAW);
			dos.flush();
			while (dis.available() <= 0) {
				Thread.sleep(10);
			}
			dis.readBoolean();
			Thread.sleep(2000);
			
			dos.writeInt(MOVE_CLAW_UP);
			dos.flush();
			while (dis.available() <= 0) {
				Thread.sleep(10);
			}
			dis.readBoolean();
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		} catch (IOException e) {
		} catch (Exception e) {
		} finally {
		}
	}
}
