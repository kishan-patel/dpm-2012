public class LightFilter extends Thread{
	
	private static int leftLight[], rightLight[],beaconLight[];
	private static int leftCurrLight, rightCurrLight,beaconCurrLight;
	public static final int sampleSize=3;
	
	/**
	 * Constructor
	 */
	public LightFilter()
	{
		leftLight = new int[sampleSize];
		rightLight = new int[sampleSize];
		beaconLight = new int[sampleSize];
		leftCurrLight=0;
		rightCurrLight=0;
		beaconCurrLight=0;
		for(int i=0; i<sampleSize; i++)
		{
			leftLight[i]=0;
			rightLight[i]=0;
			beaconLight[i]=0;
		}
	}
	
	/**
	 * @return the filtered value of the left light sensor
	 */

	public static int getLeftLight()
	{
		int sum=0;
		for(int i=0;i < sampleSize; i++)
			sum+=leftLight[i];
		return (sum/sampleSize);
	}

	/**
	 * 
	 * @return the filtered value of the right light sensor
	 */
	public static int getRightLight()
	{
		int sum=0;
		for(int i=0;i < sampleSize; i++)
			sum+=rightLight[i];
		return (sum/sampleSize);		
	}
	
	public static int getBeaconLight()
	{
		int sum=0;
		for(int i=0;i < sampleSize; i++)
			sum+=beaconLight[i];
		return (sum/sampleSize);	
	}
	
	/**
	 * 
	 * @return the filtered values of the left and right light sensors
	 */
	public static int[] getLights()
	{
		int average[]={0,0,0};
		for(int i=0;i < sampleSize; i++)
		{
			average[0]+=leftLight[i];
			average[1]+=rightLight[i];
			average[2]+=beaconLight[i];
		}
		average[0]=average[0]/sampleSize;
		average[1]=average[1]/sampleSize;
		average[2]=average[2]/sampleSize;
		return average;
		
	}
	
	/**
	 * Thread that constantly updates an array of the past few (defined by max) 
	 * light sensor values.
	 */
	public void run()
	{
		while(true)
		{
			leftLight[leftCurrLight]=SensorAndMotorInfo.LEFT_LIGHT_SENSOR.getNormalizedLightValue();
			rightLight[rightCurrLight]=SensorAndMotorInfo.RIGHT_LIGHT_SENSOR.getNormalizedLightValue();
			beaconLight[beaconCurrLight]=SensorAndMotorInfo.BEACON_FINDER_LIGHT_SENSOR.getNormalizedLightValue();

			leftCurrLight++;
			leftCurrLight%=sampleSize;
			rightCurrLight++;
			rightCurrLight%=sampleSize;
			beaconCurrLight++;
			beaconCurrLight%=sampleSize;
			try {
				Thread.sleep(Constants.LIGHT_SENSOR_SLEEP_TIME);
			} catch (InterruptedException e) {}
		}
	}
	

}