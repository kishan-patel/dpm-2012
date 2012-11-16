/**
 * @author Afif Sani 260369334
 */

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;


public class Claw {
	private final int pulleyMotorRadius = 1;
	public final static int pulleyHeight = 2;
	private final int pulleySpeed = 100;
	private final static double ONE_ROTATION = 8.4;
	private final int clawSpeed = 50;	
	private final int noOfRotsClaw = 90;	
	static final int stop = 0;

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
	this.currentHeight = pulleyHeight;

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
		int noOfRots = (int)(-150);
		moveClaw(noOfRots);
	}
	
	public void moveOffGround(){
		int noOfRots = (int)(360);
		moveClaw(noOfRots);
	}
	
	/**
	 * This method is going to open the claw if it is not open and then close it after one second. Otherwise, it will only close the claw.
	 */
	public void pickUpBeacon(){
		
		if( this.open == true ){
			
			closeClaw();
			this.open = false;

		}else{

			openClaw();
			this.open = true;			
			
		}
			
	}

	private void closeClaw(){
		clawMotor.setSpeed(clawSpeed);
		clawMotor.rotate(noOfRotsClaw + 10);
		clawMotor.setSpeed(stop);
	}
	
	public void openClaw(){
		clawMotor.setSpeed(clawSpeed);
		clawMotor.rotate(-noOfRotsClaw);
		clawMotor.setSpeed(stop);
	}
	
	private void moveClaw(int noOfRotsPulley){
		pulleyMotor.setSpeed(pulleySpeed);
		pulleyMotor2.setSpeed(pulleySpeed);	
		pulleyMotor.rotate(noOfRotsPulley,true);
		pulleyMotor2.rotate(noOfRotsPulley);;
		pulleyMotor.setSpeed(stop);
		pulleyMotor2.setSpeed(stop);
	}
	
}
