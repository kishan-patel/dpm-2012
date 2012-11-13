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
}