/**
 * @author Chris, Shravan
 * This class filters data from the USFilter. It uses a basic median filter to take a moving window filter of 5.
 */
public class USFilter extends Thread 
{
	private static int US[];
	private static int current;
	
	/**
	 * Constructor
	 */
	public USFilter()
	{
		US = new int[5];
		current=0;

		for(int i=0; i<5; i++)
		{
			US[i]=1;
		}
	}
	/**
	 * 
	 * @return the distance recorded by the UltraSonic sensor after it has been filtered
	 */
	public static int getUS()
	{
		int[] temp;
		
		temp=(int[])US.clone();
	
		return getMedian(temp);
	}
	
	/**
	 * Thread that USFilter runs in
	 */
	public void run() {
		
		while(true)
		{
			US[current]=SensorAndMotorInfo.US_SENSOR.getDistance();
			if(US[current] > Constants.US_THRESHOLD)
				US[current] = Constants.US_THRESHOLD;
			current++;
			current%=5;
			
			try {
				Thread.sleep(Constants.US_SENSOR_SLEEP_TIME);
			} catch (InterruptedException e) {}
		}
	
	}
	
	//
	/**
	 * This method returns a median of an array with and odd number of terms
	 * and values from 0 to 260 (exclusive)
	 */
	static int getMedian(int[] input)
	{
		//eliminate the max values
		for(int i = (input.length - 1)/2; i > 0; i--)
		{
			int max = 0;
			int indexOfMax = 0;
			for(int j = 0; j < input.length; j++)
			{
				if(input[j] > max)
				{
					indexOfMax = j;
					max = input[j];
				}
			}
			input[indexOfMax] = -1;
		}
		
		//eliminate the min values
		for(int i = (input.length -1)/2; i > 0; i--)
		{
			int min = 260;
			int indexOfMin = 0;
			for(int j = 0; j < input.length; j++)
			{
				if(input[j] < min && input[j] > 0)
				{
					indexOfMin = j;
					min = input[j];
				}
			}
			input[indexOfMin] = -1;
			
		}
		
		//return the median (only value left)
		for(int i = 0; i < input.length; i++)
		{
			if(input[i] != -1)
				return input[i];
		}
		
		//make eclipse stop complaining
		return 0;
	}
}
