import lejos.nxt.UltrasonicSensor;
import lejos.util.TimerListener;


public class ObstacleAvoider implements TimerListener{
	UltrasonicSensor usSensor;
	Odometer odometer;
	Navigation navigation;
	
	/**
	 * Constructor.
	 */
	public ObstacleAvoider(UltrasonicSensor usSensor, Odometer odometer, Navigation navigation){
		this.usSensor = usSensor;
		this.odometer = odometer;
		this.navigation = navigation;
	}
	
	public void timedOut(){
		
	}
	
	/**
	 * The goal of this method is to detect obstacles while navigating and avoid them.
	 */
	public void avoidObstacle(){
		
	}
}
