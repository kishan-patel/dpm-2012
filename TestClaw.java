import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.comm.Bluetooth;
import lejos.nxt.comm.NXTConnection;
import lejos.nxt.comm.RConsole;


public class TestClaw {
	public static void main(String[] args) {	
		//Variables used for by the attacker/defender.
		int buttonChoice;
		final Claw claw = new Claw(SensorAndMotorInfo.pulleyMotor, SensorAndMotorInfo.pulleyMotor2, SensorAndMotorInfo.clawMotor);
		
		LCD.drawString("Left = Move up/down", 0, 0);
		do {
			buttonChoice = Button.waitForAnyPress();
		
		} while (buttonChoice == Button.ID_LEFT);
		
		claw.moveToGround();
		try{Thread.sleep(1000);}catch(InterruptedException e){}
		claw.pickUpBeacon();
		try{Thread.sleep(1000);}catch(InterruptedException e){}
		claw.moveOffGround();
		try{Thread.sleep(1000);}catch(InterruptedException e){}
		
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		RConsole.close();
		System.exit(0);
	}
}
