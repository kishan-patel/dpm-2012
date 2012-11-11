import lejos.nxt.comm.RConsole;


public class SearchAlgorithm {
	private final int DIST_ERROR = 2;
	private int[][] fieldInfo = new int[20][20];
	
	
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
		double[] nextCoords = new double[2];
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
}
