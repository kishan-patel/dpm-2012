import lejos.nxt.comm.RConsole;


public class SearchAlgorithm {
	private final int DIST_ERROR = 2;
	private final int ANGLE_ERROR = 2;
	private int[][] fieldInfo = new int[20][20];
	private static SearchAlgorithm searchAlgorithm = null;
	//private final double[][] searchLocations = {{0.0,0.0},{90.0,0.0},{120.0,0.0},{120.0,90.0},{120.0,120.0},{90.0,120.0},{0.0,120.0},{0.0,90.0}};
	private final double[][] searchLocations = {{0.0,0.0},{30.0,30.0},{90.0,30.0},{150.0,30.0},{150.0,90.0},{150.0,150.0},{90.0,150.0},{30.0,150.0},{30.0,90.0},{30.0,30.0}};
	//private final double[][] searchLocations = {{0.0,0.0},{30.0,0.0},{60.0,0.0},{60.0,30.0},{60.0,60.0},{30.0,60.0},{0.0,60.0},{0.0,30.0}};
	private int searchLocationIndex = 1;
	private double[] nextCoords = new double[2];
	
	private SearchAlgorithm(){
	}
	
	public static SearchAlgorithm getSearchAlgorithm(){
		if(searchAlgorithm == null){
			searchAlgorithm = new SearchAlgorithm();
		}
		
		return searchAlgorithm;
	}
	
	public int[] getNextXYTile(int xCurrentTile, int xDestTile, int yCurrentTile, int yDestTile){
		fieldInfo = FieldScanner.getFieldInfo();
		int[] nextTile = new int[2];
		int xNextTile = xCurrentTile;
		int yNextTile = yCurrentTile;
		if(xCurrentTile!=xDestTile){
			xNextTile = getNextXTile(xCurrentTile, xDestTile);
		}else{
			yNextTile = getNextYTile(yCurrentTile, yDestTile);
		}
		nextTile[0] = xNextTile;
		nextTile[1] = yNextTile;

		return nextTile;
	}
	
	public double[] getNextXYCoordinate(double xCurrentCoord, double xDestCoord, double yCurrentCoord, double yDestCoord){
		double xNextCoord = xCurrentCoord;
		double yNextCoord = yCurrentCoord;
		
		if(Math.abs(xDestCoord-xCurrentCoord)>2){
			xNextCoord = getNextXCoord(xCurrentCoord, xDestCoord);
		}else if(Math.abs(yDestCoord-yCurrentCoord)>2){
			yNextCoord = getNextYCoord(yCurrentCoord, yDestCoord);
		}
		
		nextCoords[0] = xNextCoord;
		nextCoords[1] = yNextCoord;
		RConsole.println("next x = "+xNextCoord);
		RConsole.println("next y = "+yNextCoord);
		return nextCoords;
	}
	
	public double[] getNextSearchLocation(){
		int tmp = searchLocationIndex;
		searchLocationIndex++;
		if(tmp>=searchLocations.length){
			//TODO return random tile - returning for now.
			return null;
		}
		
		return searchLocations[tmp];
	}
	
	public double[] getNextLocCloserToBeacon(double xCurrentCoord,double yCurrentCoord,double theta, int distToBeacon){
		if(distToBeacon <= 100){
			//If the beacon is detected by the US sensor, then stop one tile from the beacon*/
			nextCoords[0]=(distToBeacon-30.48)*Math.cos(theta);
			nextCoords[1]=(distToBeacon-30.48)*Math.sin(theta);
		}else{
			//If the beacon is not detected by the US sensor then we move in the x and y direction
			//accordingly depending on the current angle.
			int quadrant = determineQuadrant(theta);
			
			if(quadrant == 1){
				if(Math.abs(Odometer.minimumAngleFromTo(theta, 0))<=15){
					nextCoords[0] = xCurrentCoord;
					nextCoords[1] = yCurrentCoord + (2 * 30.48);
				}else{
					nextCoords[0] = xCurrentCoord + (2 * 30.48);
					nextCoords[1] = yCurrentCoord + (2 * 30.48);
				}
			} else if (quadrant == 2) {
				if (Math.abs(Odometer.minimumAngleFromTo(theta, 90)) <= 15) {
					nextCoords[0] = xCurrentCoord + (2 * 30.48);
					nextCoords[1] = yCurrentCoord;
				} else {
					nextCoords[0] = xCurrentCoord + (2 * 30.48);
					nextCoords[1] = yCurrentCoord + (2 * 30.48);
				}
			} else if (quadrant == 3) {
				if (Math.abs(Odometer.minimumAngleFromTo(theta, 180)) <= 15) {
					nextCoords[0] = xCurrentCoord;
					nextCoords[1] = yCurrentCoord - (2*30.48);
				} else {
					nextCoords[0] = xCurrentCoord + (2 * 30.48);
					nextCoords[1] = yCurrentCoord + (2 * 30.48);
				}
			} else {
				if (Math.abs(Odometer.minimumAngleFromTo(theta, 270)) <= 15) {
					nextCoords[0] = xCurrentCoord - (2 * 30.48);
					nextCoords[1] = yCurrentCoord;
				} else {
					nextCoords[0] = xCurrentCoord + (2 * 30.48);
					nextCoords[1] = yCurrentCoord + (2 * 30.48);
				}
			}
		}
		
		return nextCoords;
	}
	
	private int getNextXTile(int xCurrentTile, int destXTile){
		int nextXTile = destXTile;
		return nextXTile;
	}
	
	private int getNextYTile(int xCurrentTile, int destYTile){
		int nextYTile = destYTile;
		return nextYTile;
	}
	
	private double getNextXCoord(double xCurrentCoord, double xDestCoord){
		double xNextCoord = xDestCoord;
		return xNextCoord;
	}
	
	private double getNextYCoord(double yCurrentCoord, double yDestCoord){
		double yNextCoord = yDestCoord;
		return yNextCoord;
	}
	
	private int determineQuadrant(double theta){
		if(Math.abs(Odometer.minimumAngleFromTo(theta, 0))<=5&&Math.abs(Odometer.minimumAngleFromTo(theta, 90))<=5){
			return 1;
		}else if (Math.abs(Odometer.minimumAngleFromTo(theta, 90))<=5&&Math.abs(Odometer.minimumAngleFromTo(theta, 180))<=5){
			return 2;
		}else if (Math.abs(Odometer.minimumAngleFromTo(theta, 90))<=5&&Math.abs(Odometer.minimumAngleFromTo(theta, 180))<=5){
			return 3;
		}else{
			return 4;
		}
	}
}
