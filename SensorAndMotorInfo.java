import lejos.nxt.LightSensor;
import lejos.nxt.SensorPort;

public class SensorAndMotorInfo {
	private final static SensorPort US_SENSOR_PORT = SensorPort.S1;
	private final static SensorPort LEFT_LIGHT_SENSOR_PORT = SensorPort.S2;
	private final static SensorPort RIGHT_LIGHT_SENSOR_PORT = SensorPort.S2;
	private final static SensorPort BEACON_FINDER_LIGHT_SENSOR_PORT = SensorPort.S4;
	private final static USSensor US_SENSOR = new USSensor(US_SENSOR_PORT);
	private final static LightSensor LEFT_LIGHT_SENSOR = new LightSensor(LEFT_LIGHT_SENSOR_PORT);
	private final static LightSensor RIGHT_LIGHT_SENSOR = new LightSensor(RIGHT_LIGHT_SENSOR_PORT);
	private final static LightSensor BEACON_FINDER_LIGHT_SENSOR = new LightSensor(BEACON_FINDER_LIGHT_SENSOR_PORT);

	public static USSensor getUsSensor() {
		return US_SENSOR;
	}
	
	public static LightSensor getLeftLightSensor(){
		return LEFT_LIGHT_SENSOR;
	}
	
	public static LightSensor getRightLightSensor(){
		return RIGHT_LIGHT_SENSOR;
	}
	
	public static LightSensor getBeaconFinderLightSensor(){
		return BEACON_FINDER_LIGHT_SENSOR;
	}
}
