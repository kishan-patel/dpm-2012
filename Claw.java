import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;


public class Claw {
	private final int pulleyMotorRadius = (int)(2.10/2);
	private final int pulleyHeight = 12;
	private final int pulleySpeed = 100;
	private final int clawSpeed = 10;
	private final int noOfRotsPulley = (int) (360*pulleyHeight/(2*Math.PI*pulleyMotorRadius));
	private final int noOfRotsClaw = 80;
	private NXTRegulatedMotor clawMotor = Motor.B;
	private NXTRegulatedMotor pulleyMotor = Motor.A;
	final static int stop = 0;

	private boolean down = false;
	private boolean open = false;
	
	public void closeClaw(){
		clawMotor.setSpeed(clawSpeed);
		clawMotor.rotate(noOfRotsClaw + 10);
		clawMotor.setSpeed(stop);
	}
	
	public void openClaw(){
		clawMotor.setSpeed(clawSpeed);
		clawMotor.rotate(-noOfRotsClaw);
		clawMotor.setSpeed(stop);
	}
	
	public void moveClawUp(){
		pulleyMotor.setSpeed(pulleySpeed);
		pulleyMotor.rotate(noOfRotsPulley);
		pulleyMotor.setSpeed(stop);
	}
	
	public void moveClawDown(){
		pulleyMotor.setSpeed(pulleySpeed);
		pulleyMotor.rotate(-noOfRotsPulley);
		pulleyMotor.setSpeed(stop);
	}

	
	public static void main(String[] args){
		Claw claw = new Claw();
		int buttonChoice;
		
		LCD.clear();
		LCD.drawString("Left: Move up/down",0,0);
		LCD.drawString("Right: Close or Open",0,1);
		do{
			buttonChoice = Button.waitForAnyPress();
			// Pulling the claw up or down
			if(buttonChoice==Button.ID_LEFT){
				if(claw.down){
					claw.moveClawUp();
					claw.down = false;
				}else{
					claw.moveClawDown();
					claw.down = true;
				}
			// Setting the claw close or open
			}else if(buttonChoice == Button.ID_RIGHT){
				if(claw.open){
					claw.closeClaw();
					claw.open = false;					
				}else{
					claw.openClaw();
					claw.open = true;
				}
			}
		}while(buttonChoice!=Button.ID_ESCAPE);
	}
}
