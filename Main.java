import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
import lejos.util.Timer;
import lejos.util.TimerListener;


public class Main {

	public static void main(String[] args){
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S1);
		LightSensor ls = new LightSensor(SensorPort.S2);
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		Odometer odo = new Odometer(patBot, true);
		LCDInfo lcd = new LCDInfo(odo);
		USLocalizer usl = new USLocalizer(odo, us,
				USLocalizer.LocalizationType.FALLING_EDGE);
		LightFinder lf = new LightFinder(odo, us, ls);
		Navigation nav = new Navigation(odo);
		OdometryCorrector odometryCorrection = new OdometryCorrector(odo);
		BrickCommunicator brickCommunicator = new BrickCommunicator();
		BeaconLifter beaconLifter = new BeaconLifter();
		SearchAlgo sAlgo = new SearchAlgo();
		boolean beaconFound = false;
		TimerListener hcl = new HeadingCorrector();
		TimerListener ocl = new OdometryCorrector(odo);
		TimerListener oal = new ObstacleAvoider(us,odo,nav);
		Timer hc;
		Timer oc;
		Timer oa;
		Timer ab;
		
		while(!beaconFound){
			lf.findLightSourceHeading();
			if(lf.LightSourceFound()){
				lf.turnToLightSourceHeading();
				hc = new Timer(10,hcl);
				oc = new Timer(10,ocl);
				oa = new Timer(10,oal);
				hc.start();
				oc.start();
				oa.start();
			}else{
				sAlgo.navigateToNextCoordinate();
			}
		}
	}
}
