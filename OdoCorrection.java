import lejos.nxt.LightSensor;
import lejos.nxt.Sound;
import lejos.nxt.comm.RConsole;
import lejos.util.TimerListener;

/**
 *   
 * This method corrects the odometer's values (x, y and theta) when the robot crosses a line
 */
public class OdoCorrection implements TimerListener
{
	/**
	 * 
	 * @return boolean array representing which light sensors are on lines (0 = left, 1 = right)
	 */
	
	private final int timedOut=500;
	private boolean disabled = false;
	private Odometer odo; 
	LightSensor leftLightSensor;
	LightSensor rightLightSensor;
	private double deltaTheta = 0.0;
	private Object lock = new Object();
	public int rightSensorTime=0;
	public int leftSensorTime=0;

	public OdoCorrection(Odometer odo){
		this.odo = odo;
		leftLightSensor = SensorAndMotorInfo.LS_LEFT_SENSOR;
		rightLightSensor = SensorAndMotorInfo.LS_RIGHT_SENSOR;
	}
	
	private  boolean[] isOnLine()
	{
		boolean a[]= new boolean[2];
		a[0]= leftLightSensor.getLightValue()< 45;
		a[1]= rightLightSensor.getLightValue()< 45;
		return a;
	}
	
	/**
	 * constantly checks whether the robot has crossed a line and calls correct() when it does.
	 */
	public void timedOut()
	{
		
		
		long time1, time2, timeRight, timeLeft;// deltaTime=0;
		int temp;
		//long wait;
		boolean[] det;
		while(true)
		{
			
			det=isOnLine();
			rightSensorTime = 0;
			leftSensorTime = 0;
			if( (det[0] == true || det[1]==true) && (det[0]!=det[1]) )//xor
			{
			
				time1=System.currentTimeMillis();
				
				temp=1;
				if(det[0]==false)
					temp=0;
				
				time2=System.currentTimeMillis();
				while( det[temp]==false && time2 - time1<timedOut)
				{
					det=isOnLine();
					time2=System.currentTimeMillis();					
				}
				if(time2 - time1<timedOut)
				{
				
					//time2=wait;
		
					if(temp==0)
					{
						//Right crosses before left
						rightSensorTime = 0;
						leftSensorTime = 1;
						timeRight=time1;
						timeLeft=time2;
					}
					else
					{
						rightSensorTime = 1;
						leftSensorTime = 0;
						timeLeft=time1;
						timeRight=time2;
					}		
					
					correct(timeLeft - timeRight);
				}				
			}
			else if(det[0]==true && det[1]==true)
			{
				correct(0);
			}				
		}
	}
		
	/**
	 * corrects the odometer's values after crossing a line 
	 * @param deltaTime difference in time between when each light sensor detected a line
	 */
	public void correct(long deltaTime)
	{
		synchronized (lock) {
			Coordinates current = odo.getCoordinates();
			double[] position = new double[3];
			odo.getPosition(position);
			double distance,averageSpeed,axis;
			double deltaTheta;
			
			//averageSpeed=(SysConst.LEFT_RADIUS*SysConst.leftMotor.getSpeed()+SysConst.RIGHT_RADIUS*SysConst.rightMotor.getSpeed())*(Math.PI/180.0)/2; 
			averageSpeed=TwoWheeledRobot.DEFAULT_LEFT_RADIUS*TwoWheeledRobot.leftMotor.getSpeed()*(Math.PI/180.0);
			distance=deltaTime*averageSpeed/1000.0;
			deltaTheta = Math.atan(distance/TwoWheeledRobot.DEFAULT_WIDTH);
			RConsole.println("Delta theta is: "+deltaTheta);
		}

		
		/*if (!disabled && (Math.abs(current.x / 30.48) < 5 || Math.abs(current.y / 30.48) < 5)) 
		{

			if (current.theta > 50 && current.theta < 130) {
				current.x = 3.12+position[0]- (30.48 - position[0] % 30.48) * Math.tan(deltaTheta);
				current.theta = 90 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.y / 30.48);
				current.y = axis
						+ ((TwoWheeledRobot.DEFAULT_WIDTH / 2.0) * Math.sin(Math
								.abs(deltaTheta)));
				//correct the X position assuming a constant "deltatTheta" over the whole score, correcting for the whole distance (kinda in advance of 15.24cm)
			} else if (current.theta > 230 && current.theta < 310) {
				current.x = 3.12+position[0] + (position[0] % 30.48) * Math.tan(deltaTheta);
				current.theta = 270 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.y / 30.48);
				current.y = axis
						- ((TwoWheeledRobot.DEFAULT_WIDTH / 2.0) * Math.sin(Math
								.abs(deltaTheta)));

			} else if (current.theta > 140 && current.theta < 220) {
				current.y = 3.12+position[1] - (position[1] % 30.48) * Math.tan(deltaTheta);
				current.theta = 180 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.x / 30.48);
				current.x = axis
						- ((TwoWheeledRobot.DEFAULT_WIDTH / 2.0) * Math.sin(Math
								.abs(deltaTheta)));

			} else if (current.theta > 320 || current.theta < 40) {
				current.y = 3.12+position[1] + (30.48 - position[1] % 30.48) * Math.tan(deltaTheta);
				current.theta = 0 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.x / 30.48);
				current.x = axis
						+ ((TwoWheeledRobot.DEFAULT_WIDTH / 2.0) * Math.sin(Math
								.abs(deltaTheta)));
			}
			current.theta += 360;
			current.theta %= 360;
			//current.theta = odo.getTheta();
			RConsole.println("current x: "+current.x);
			RConsole.println("current y: "+current.y);
			RConsole.println("current theta: "+current.theta);
			odo.setCoordinates(current, new boolean[] { true, true, true });*/
			Sound.buzz();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {}
		}	
	
	public void resetDeltaTheta(){
		synchronized (lock) {
			deltaTheta = 0;
		}
	}
	
	public double getDeltaTheta(){
		synchronized (lock) {
			return deltaTheta;
		}
	}
}
	

