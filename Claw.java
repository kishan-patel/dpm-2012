/**
 * @author Afif Sani 260369334
 */

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;


public class Claw {
	private final int PULLEY_MOTOR_RADIUS = 1;
	public final static int PULLEY_HEIGHT = 19;
	private final int PULLEY_SPEED = 100;
	private final static double ONE_ROTATION = 8.4;
	private final int CLAW_SPEED = 50;	
	private final int NO_OF_ROTS_CLAW = 50;	
	static final int STOP = 0;

	private NXTRegulatedMotor clawMotor;
	private NXTRegulatedMotor pulleyMotor;
	private NXTRegulatedMotor pulleyMotor2;
	//private NXTRegulatedMotor clawMotor = Motor.B;
	//private NXTRegulatedMotor pulleyMotor = Motor.A;	
	private boolean open;
	private int currentHeight;
	
	/**
	 * Constructor.
	 */
	public Claw(NXTRegulatedMotor pulleyMotor, NXTRegulatedMotor pulleyMotor2, NXTRegulatedMotor clawMotor){
	
	this.clawMotor = clawMotor;
	this.pulleyMotor = pulleyMotor;
	this.pulleyMotor2 = pulleyMotor2;
	this.open = true;
	this.currentHeight = PULLEY_HEIGHT;

	}
	
	/**
	 * This method is going to move the pulley to the height given.
	 */
	public void moveToHeight(int height){
		
		int oldHeight = height;		
		height = height - this.currentHeight;		
		this.currentHeight = oldHeight;		
		
		int noOfRotsPulley = (int) (360.0*(double)height/ONE_ROTATION);				

					try {				
						Thread.sleep(100);			
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}					
					
					moveClaw(noOfRotsPulley);
									
	}
	
	public void moveToGround(){
		moveToHeight(STOP);
	}
	
	public void moveOffGround(){
		moveToHeight(PULLEY_HEIGHT);
	}
	
	/**
	 * This method is going to open the claw if it is not open and then close it after one second. Otherwise, it will only close the claw.
	 */
	public void pickUpBeacon(){
		
		if( this.open == true ){
			
			closeClaw();
			this.open = false;

		}
			
	}

	private void closeClaw(){
		clawMotor.setSpeed(CLAW_SPEED);
		clawMotor.rotate(NO_OF_ROTS_CLAW+35);
		clawMotor.setSpeed(STOP);
	}
	
	public void openClaw(){
		clawMotor.setSpeed(CLAW_SPEED);
		clawMotor.rotate(-NO_OF_ROTS_CLAW);
		clawMotor.setSpeed(STOP);
		this.open = true;
	}
	
	private void moveClaw(int noOfRotsPulley){
		pulleyMotor.setSpeed(PULLEY_SPEED);
		pulleyMotor2.setSpeed(PULLEY_SPEED);	
		pulleyMotor.rotate(noOfRotsPulley,true);
		pulleyMotor2.rotate(noOfRotsPulley);;
		pulleyMotor.setSpeed(STOP);
		pulleyMotor2.setSpeed(STOP);
	}
	
}
