import lejos.nxt.LightSensor;
import lejos.nxt.Sound;
import lejos.nxt.comm.RConsole;
import lejos.util.TimerListener;
import lejos.util.Timer;

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
	private static final int timedOut=1500;
	public static boolean disabled = false;
	private static LightSensor leftLS = SensorAndMotorInfo.LEFT_LIGHT_SENSOR;
	private static LightSensor rightLS = SensorAndMotorInfo.RIGHT_LIGHT_SENSOR;
	private static int leftTresh = 47;
	private static int rightTresh = 49;
	private static Odometer odo;
	static int leftLV;
	static int rightLV;
	static int leftCount = 0;
	static int rightCount = 0;
	Timer timer;
	
	public OdoCorrection(Odometer odometer){
		odo = odometer;
	}
	public void startCorrection(){
		timer = new Timer(10,this);
		timer.start();
	}
	
	public void stopTimer(){
		timer.stop();
	}
	
	private static boolean[] isOnLine()
	{
		boolean a[]= new boolean[2];
		int l[]=LightFilter.getLights();
		a[0]= l[0]< Constants.LEFT_LIGHT_THRESHOLD;
		a[1]= l[1]< Constants.RIGHT_LIGHT_THRESHOLD;
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
		
			det=isOnLine();
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
						timeRight=time1;
						timeLeft=time2;
					}
					else
					{
						timeLeft=time1;
						timeRight=time2;
					}		
					
					RConsole.println("correcting theta");
					correct(timeLeft - timeRight);
				}				
			}
			else if(det[0]==true && det[1]==true)
			{
				correct(0);
			}				
		
	}
		
	/**
	 * corrects the odometer's values after crossing a line 
	 * @param deltaTime difference in time between when each light sensor detected a line
	 */
	public static void correct(long deltaTime)
	{
		Coordinates current=Odometer.getCoordinates();
		double distance,averageSpeed,axis;
		double deltaTheta;
		
		//averageSpeed=(SysConst.LEFT_RADIUS*SysConst.leftMotor.getSpeed()+SysConst.RIGHT_RADIUS*SysConst.rightMotor.getSpeed())*(Math.PI/180.0)/2; 
		averageSpeed=TwoWheeledRobot.leftRadius*TwoWheeledRobot.leftMotor.getSpeed()*(Math.PI/180.0);
		RConsole.println("time: "+deltaTime);
		distance=deltaTime*averageSpeed/1000.0;
		deltaTheta = Math.atan(distance/TwoWheeledRobot.width);
		RConsole.println("D-theta: "+deltaTheta);
		deltaTheta *= -1;

		
		if (!disabled && (Math.abs(current.x / 30.48) < 5 || Math.abs(current.y / 30.48) < 5)) 
		{
			if (Math.abs(Odometer.minimumAngleFromTo(current.theta, 90))<=5) {
				current.x = Navigation.initPoint.x - (30.48 - Navigation.initPoint.y % 30.48) * Math.tan(deltaTheta);
				current.theta = 90 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.y / 30.48);
				current.y = axis
						+ ((TwoWheeledRobot.width / 2.0) * Math.sin(Math
								.abs(deltaTheta)));
				//correct the X position assuming a constant "deltatTheta" over the whole score, correcting for the whole distance (kinda in advance of 15.24cm)
			} else if (Math.abs(Odometer.minimumAngleFromTo(current.theta, 270))<=5) {
				current.x = Navigation.initPoint.x + (Navigation.initPoint.y % 30.48) * Math.tan(deltaTheta);
				current.theta = 270 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.y / 30.48);
				current.y = axis
						- ((TwoWheeledRobot.width / 2.0) * Math.sin(Math
								.abs(deltaTheta)));

			} else if (Math.abs(Odometer.minimumAngleFromTo(current.theta, 180))<=5) {
				current.y = Navigation.initPoint.y - (Navigation.initPoint.x % 30.48) * Math.tan(deltaTheta);
				current.theta = 180 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.x / 30.48);
				current.x = axis
						- ((TwoWheeledRobot.width / 2.0) * Math.sin(Math
								.abs(deltaTheta)));

			} else if (Math.abs(Odometer.minimumAngleFromTo(current.theta, 0))<=5) {
				current.y = Navigation.initPoint.y + (30.48 - Navigation.initPoint.x % 30.48) * Math.tan(deltaTheta);
				current.theta = 0 + deltaTheta * (180.0 / Math.PI);
				axis = 30.48 * Math.round(current.x / 30.48);
				current.x = axis
						+ ((TwoWheeledRobot.width / 2.0) * Math.sin(Math
								.abs(deltaTheta)));
			}
			/*current.theta += 360;
			current.theta %= 360;*/
			current.theta = (odo.getTheta()+deltaTheta * (180.0 / Math.PI));
			current.x = odo.getXPos();
			current.y = odo.getYPos();
			RConsole.println("curr. theta = " + odo.getTheta());
			RConsole.println("corr.theta = "+current.theta);
			Odometer.setCoordinates(current, new boolean[] { true, true, true });
			Sound.buzz();
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {}
		}	
	}	
}
	

