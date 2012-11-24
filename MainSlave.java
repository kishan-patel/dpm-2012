/**
 * @author Afif Sani 260369334
 */

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.*;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.RConsole;
import lejos.util.Delay;
import lejos.nxt.Button;



public class MainSlave {
	public static final int CLOSE_CLAW = 0;
	public static final int OPEN_CLAW = 1;
	public static final int LOWER_CLAW_TO_FLOOR = 2;
	public static final int MOVE_CLAW_UP = 3;
	
	/**
	 * @param args
	 */
	private static NXTConnection connectionToMaster;
	private static DataInputStream dis;
	private static DataOutputStream dos;	

	public static void main(String[] args) {
		int buffer = 0;		
		final Claw claw = new Claw(SensorAndMotorInfo.pulleyMotor, SensorAndMotorInfo.pulleyMotor2, SensorAndMotorInfo.clawMotor);
		// Exit program when escape button is pressed
		(new Thread() {
			public void run() {
				int choice = Button.waitForAnyPress();
				while (choice != Button.ID_ESCAPE){
					choice = Button.waitForAnyPress();
				}
					
			}
		}).start();
		
		LCD.drawString("Starting BT connection", 0, 0);
		// Starting the input and output stream
		connectionToMaster = Bluetooth.waitForConnection();
		dis = connectionToMaster.openDataInputStream();
		LCD.drawString("Opened DIS", 0, 1);
		dos = connectionToMaster.openDataOutputStream();
		LCD.drawString("Opened DOS", 0, 2);
		
		(new Thread(){
			public void run(){
				int buff;
				while(true){
					try {
						while (dis.available() <= 0) {
							Thread.sleep(10);
						}
						
						buff = dis.readInt();
						if(buff == CLOSE_CLAW){
							claw.pickUpBeacon();
							dos.writeBoolean(true);
							dos.flush();
						}else if(buff == OPEN_CLAW){
							claw.openClaw();
							dos.writeBoolean(true);
							dos.flush();
						}else if(buff == LOWER_CLAW_TO_FLOOR){
							claw.moveToGround();
							dos.writeBoolean(true);
							dos.flush();
						}else if(buff == MOVE_CLAW_UP){
							claw.moveOffGround();
							dos.writeBoolean(true);
							dos.flush();
						}
						
						
					} catch (InterruptedException | IOException e) {

					}
				}
			}
		}).start();
		
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		try {
			dos.close();
			dis.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		RConsole.close();
		System.exit(0);
//		
//		// Data stream for opening the claw
//		try {// Waiting if there is a data stream available
//			while (dis.available() <= 0)
//				Thread.sleep(10);
//				
//			// Read the data stream
//			buffer = dis.readInt();
//		
//			LCD.drawString("Received information", 0, 3);
//			LCD.drawInt(buffer, 0, 5);
//			LCD.drawString("Sent confirmation", 0, 4);
//			// This is going to open the claw
//			if(buffer == -1){
//				
//				claw.openClaw();	
//					
//			}
//			// Feedback for master
//			dos.writeBoolean(true);
//			dos.flush();
//
//			
//		} catch (IOException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		} catch (InterruptedException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		}
//	
//		
//		// Sleep for a while for confirmation
//		try {				
//						Thread.sleep(2000);			
//					} catch (InterruptedException e) {
//						// TODO Auto-generated catch block
//						e.printStackTrace();
//					}	
//					
//				LCD.clear(3);
//				LCD.clear(4);
//				LCD.clear(5);
//		
//		// Data stream for moving the claw to certain height 			
//		try {// Waiting if there is a data stream available
//			while (dis.available() <= 0)
//				Thread.sleep(10);
//				
//			// Read the data stream
//			buffer = dis.readInt();
//			
//		
//			LCD.drawString("Received information", 0, 3);			
//			LCD.drawInt(buffer, 0, 5);
//			LCD.drawString("Sent confirmation", 0, 4);
//			
//			// This is going to grab the beacon
//			if(buffer > -1){
//				
//				claw.moveToHeight(buffer);
//				claw.pickUpBeacon();
//				claw.moveToHeight(Claw.pulleyHeight);
//					
//			}
//			// Feedback for master
//			dos.writeBoolean(true);			
//			dos.flush();
//
//			
//		} catch (IOException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		} catch (InterruptedException e1) {
//			// TODO Auto-generated catch block
//			e1.printStackTrace();
//		}		
//		
//		// Sleep for a while for confirmation
//		try {				
//						Thread.sleep(2000);			
//					} catch (InterruptedException e) {
//						// TODO Auto-generated catch block
//						e.printStackTrace();
//					}	
//					
//				LCD.clear(3);
//				LCD.clear(4);
//				LCD.clear(5);
//		
//		// Data stream to release the claw
//		try {// Waiting if there is a data stream available
//			while (dis.available() <= 0)
//				Thread.sleep(10);
//				
//			// Read the data stream
//			buffer = dis.readInt();
//		
//			LCD.drawString("Received information", 0, 3);
//			LCD.drawInt(buffer, 0, 5);
//			LCD.drawString("Sent confirmation", 0, 4);
//			// This is going to open the claw
//			if(buffer == -1){
//				
//				claw.openClaw();	
//					
//			}
//			// Feedback for the master
//			dos.writeBoolean(true);
//			dos.flush();
//		
//		while(true){
//			try {dis.close();} 
//			catch (Exception e ) {System.out.println(" close error "+e);} 
//
//			try {dos.close();} 
//			catch (Exception e ) {System.out.println(" close error "+e);}
//
//			try {connectionToMaster.close();	} 
//			catch (Exception e ) {System.out.println(" close error "+e);}
//
//			break;
//		}
//
//	
//		
//
//	}catch(Exception e){
//		
//	}
//	}
	}
}

