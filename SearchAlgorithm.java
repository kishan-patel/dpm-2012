import lejos.nxt.comm.RConsole;


public class SearchAlgorithm {
	private static SearchAlgorithm searchAlgorithm = null;
	private final double[][] attackerSearchBox = {{0.0,0.0},{30.0,0.0},{60.0,0.0},{90.0,0.0},{90.0,30.0},{90.0,60.0},{90.0,90},{60.0,90.0},{30.0,90.0},{0,90.0},{0.0,0.0}};
	private double[][] defenderSearchBox;
	private int currentAttackerIndex = 0;
	private int nextAttackerIndex = 0;
	private int currentDefenderIndex = 0;
	private int nextDefenderIndex = 0;
	
	private double[] nextCoords = new double [2];

	
	private SearchAlgorithm(){
	}
	
	public static SearchAlgorithm getSearchAlgorithm(){
		if(searchAlgorithm == null){
			searchAlgorithm = new SearchAlgorithm();
		}
		
		return searchAlgorithm;
	}
	
	
	public double[] getNextAttackerSearchLocation(){
		if(currentAttackerIndex>=attackerSearchBox.length){
			//TODO return random tile - returning for now.
			return null;
		}
		currentAttackerIndex = nextAttackerIndex;
		nextAttackerIndex++;
		return attackerSearchBox[currentAttackerIndex];
	}	
	
	public double[] getNextDefenderSearchLocation(){
		if(currentDefenderIndex>=defenderSearchBox.length){
			//TODO return random tile - returning for now
			return null;
		}
		currentDefenderIndex = nextDefenderIndex;
		nextDefenderIndex++;
		return defenderSearchBox[currentDefenderIndex];
	}
	
	public void setDefenderLocation(int tileX, int tileY){
		double xCoord = tileX * 30.48;
		double yCoord = tileY * 30.48;
		
		defenderSearchBox = new double[][]
				{
					{yCoord - 30.48, xCoord},
					{yCoord - 30.48, xCoord+30.48},
					{yCoord, xCoord + 30.48},
					{yCoord+30.48, xCoord + 30.48},
					{yCoord+30.48, xCoord},
					{yCoord+30.48, xCoord - 30.48},
					{yCoord, xCoord - 30.48},
					{yCoord - 30.48, xCoord - 30.48},
				};
	}
	
	public void markCurrentAttackerLocationBlocked(){
		currentAttackerIndex = nextAttackerIndex;
		nextAttackerIndex++;
	}
	
	public void markCurrentDefenderLocationAsBlocked(){
		currentDefenderIndex = nextDefenderIndex;
		nextDefenderIndex++;
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
	
	private double getNextXCoord(double xCurrentCoord, double xDestCoord){
		double xNextCoord = xDestCoord;
		return xNextCoord;
	}
	
	private double getNextYCoord(double yCurrentCoord, double yDestCoord){
		double yNextCoord = yDestCoord;
		return yNextCoord;
	}
}
