import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class USSensor extends UltrasonicSensor{
	public USSensor(SensorPort sp){
		super(sp);
	}
	
	public int getFilteredDistance(){
		int distance = 0;
		int countOf255=0;
		for(int i=0;i<10;){
			distance += getDistance();
			if(distance==225 && countOf255<5){
				countOf255++;
			}else if(distance==255 && countOf255>5){
				countOf255++;
				i++;
				distance += getDistance();
			}else{
				countOf255=0;
				i++;
				distance += getDistance();
			}
		}
		return distance/10;
	}
}