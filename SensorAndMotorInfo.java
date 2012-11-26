import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.SensorPort;

public class SensorAndMotorInfo {
	public final static SensorPort US_SENSOR_PORT = SensorPort.S3;
	public final static SensorPort LS_LOCALIZATION_SENSOR_PORT = SensorPort.S1;
	public final static SensorPort BEACON_FINDER_LIGHT_SENSOR_PORT = SensorPort.S4;
	public final static USSensor US_SENSOR = new USSensor(US_SENSOR_PORT);
	public final static LightSensor LS_LOCALIZATION_SENSOR = new LightSensor(LS_LOCALIZATION_SENSOR_PORT);
	public final static LightSensor BEACON_FINDER_LIGHT_SENSOR = new LightSensor(BEACON_FINDER_LIGHT_SENSOR_PORT);
	public final static NXTRegulatedMotor clawMotor = Motor.B;
	public final static NXTRegulatedMotor pulleyMotor = Motor.A;
	public final static NXTRegulatedMotor pulleyMotor2 = Motor.C;
	public final static LightSensor LEFT_LIGHT_SENSOR = new LightSensor(SensorPort.S1);
	public final static LightSensor RIGHT_LIGHT_SENSOR = new LightSensor(SensorPort.S2);
	
}
