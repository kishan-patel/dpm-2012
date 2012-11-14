/**
 * @author Afif Sani 260369334
 */

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;


public class Claw {
	private final int pulleyMotorRadius = 1;
	private final int pulleyHeight = 12;
	private final int pulleySpeed = 100;
	//private final int noOfRotsPulley = (int) (360*pulleyHeight/(2*Math.PI*pulleyMotorRadius)); // The value is in angle, 360=1 rotation, 720=2 rotations
	private final int clawSpeed = 10;	
	private final int noOfRotsClaw = 80;	
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
	this.open = false;
	this.currentHeight = pulleyHeight;

	}
	
	/**
	 * This method is going to move the pulley to the height given.
	 */
	public void moveToHeight(int height){
		
		int oldHeight = height;		
		height = height - this.currentHeight;		
		this.currentHeight = oldHeight;		
		
		int noOfRotsPulley = (int) (360*height/(2*Math.PI*pulleyMotorRadius));				

					try {				
						Thread.sleep(100);			
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}					
					
					moveClaw(noOfRotsPulley);
									
	}
	
	/**
	 * This method is going to open the claw if it is not open and then close it after one second. Otherwise, it will only close the claw.
	 */
	public void pickUpBeacon(){
		
		if( this.open ){
			
			closeClaw();
			this.open = false;

		}else{

			openClaw();
			this.open = true;

			try {				
				Thread.sleep(1000);			
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			closeClaw();
			this.open = false;
			
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
