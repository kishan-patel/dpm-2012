import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;

public class SensorAndMotorInfo {
	public final static SensorPort US_SENSOR_PORT = SensorPort.S3;
	public final static SensorPort LEFT_LIGHT_SENSOR_PORT = SensorPort.S2;
	public final static SensorPort RIGHT_LIGHT_SENSOR_PORT = SensorPort.S1;
	public final static SensorPort BEACON_FINDER_LIGHT_SENSOR_PORT = SensorPort.S4;
	public final static USSensor US_SENSOR = new USSensor(US_SENSOR_PORT);
	public final static LightSensor LEFT_LIGHT_SENSOR = new LightSensor(LEFT_LIGHT_SENSOR_PORT);
	public final static LightSensor RIGHT_LIGHT_SENSOR = new LightSensor(RIGHT_LIGHT_SENSOR_PORT);
	public final static LightSensor BEACON_FINDER_LIGHT_SENSOR = new LightSensor(BEACON_FINDER_LIGHT_SENSOR_PORT);
	public final static NXTRegulatedMotor clawMotor = Motor.B;
	public final static NXTRegulatedMotor pulleyMotor = Motor.A;
	public final static NXTRegulatedMotor pulleyMotor2 = Motor.C;

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
