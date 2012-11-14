import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class USSensor extends UltrasonicSensor{
	private final int FILTER_OUT = 20;
	int filterControl = 0;
	int distance = 0;
	
	public USSensor(SensorPort sp){
		super(sp);
	}
	
	public int getFilteredDistance(){
		int currentDistance = getDistance();
		if (currentDistance == 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		}  else {
			// distance went below 50, therefore reset everything.
			filterControl = 0;
			distance = currentDistance;
		}
		
		return currentDistance;
	}
	
	public int getDistanceToObstacle(){
		int noOfObjectDetections = 0;
		int currentDistance=0;
		int totalDistance=0;
		
		for(int i=0; i<10; i++){
			currentDistance = getDistance();
			if(currentDistance<=100){
				noOfObjectDetections++;
				totalDistance+=currentDistance;
			}
			try{
				Thread.sleep(10);
			}catch(InterruptedException e){
			}
		}
		
		if(noOfObjectDetections>7){
			return totalDistance/noOfObjectDetections;
		}else{
			return 255;
		}
	}
}