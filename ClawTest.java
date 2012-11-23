import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.RConsole;


public class ClawTest {
	public static void main(String[] args) {
		int buttonChoice;
		//Variables used for communication with slave.
		NXTConnection connectionToSlave;
		DataInputStream dis;
		DataOutputStream dos;
		final int CLOSE_CLAW = 0;
		final int OPEN_CLAW = 1;
		int LOWER_CLAW_TO_FLOOR = 2;
		int MOVE_CLAW_UP = 3;
		
		//Variables used for by the attacker/defender.
		USSensor  usSensor = SensorAndMotorInfo.US_SENSOR;;
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);;
		Odometer odo = new Odometer(patBot,true);;
		OdoCorrection odoCorrection = new OdoCorrection(odo);;
		Navigation  nav = Navigation.getNavigation(odo);
		FieldScanner  fieldScanner = FieldScanner.getFieldScanner(odo);;
		SearchAlgorithm searchAlgorithm = SearchAlgorithm.getSearchAlgorithm();;
		USLocalizer usl = usl = new USLocalizer(odo, usSensor, USLocalizer.LocalizationType.FALLING_EDGE);;
		
	
		do {
			LCD.drawString("Left - Run test", 0, 0);
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT);
		
		try {
			connectionToSlave = Bluetooth.connect("ShootingBrick",NXTConnection.PACKET);
			dis = connectionToSlave.openDataInputStream();
			dos = connectionToSlave.openDataOutputStream();
		
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
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	}
}
