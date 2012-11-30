import lejos.nxt.comm.RConsole;


public class SearchAlgorithm {
	private static SearchAlgorithm searchAlgorithm = null;
	//private final double[][] attackerSearchBox = {{0.0,0.0},{30.0,0.0},{60.0,0.0},{90.0,0.0},{90.0,30.0},{90.0,60.0},{90.0,90},{60.0,90.0},{30.0,90.0},{0,90.0},{0.0,0.0}};
	private final double[][] attackerSearchBox = {
			{30.48, 91.44},
			{30.48, 213.36},
			{152.4, 274.32},
			{213.36,274.32},
			{213.36,91.44},
			{152.4, 91.44},
			
			{30.48,30.48},
			{30.48,274.32},
			{274.32,274.32},
			{274.32,30.48},
			/*
			{213.36, 152.4},
			{152.4, 213.36},
			{30.48,152.4},
			{152.4,274.32}, 
			{274.32,152.4},
			{152.4,30.48},
			{152.4,152.4},
			*/
			
	};
	private double[][] defenderSearchBox;/* = {{15.24,15.24},{45.72,15.24},{76.2,15.24},{76.2,45.72},{76.2,76.2},{45.72,76.2},{15.24,76.2},{15.24,45.72}};*/
	private int currentAttackerIndex = 0;
	private int nextAttackerIndex = 0;
	private int currentDefenderIndex = 0;
	private int nextDefenderIndex = 0;
	public static final int TRAVEL_IN_X = 1;
	public static final int TRAVEL_IN_Y = 2;
	public static int nextToTravel = TRAVEL_IN_X;
	
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
		if(nextAttackerIndex>=attackerSearchBox.length){
			//TODO return random tile - returning for now.
			return null;
		}
		currentAttackerIndex = nextAttackerIndex;
		nextAttackerIndex++;
		return attackerSearchBox[currentAttackerIndex];
	}	
	
	public double[] getNextDefenderSearchLocation(){
		if(nextDefenderIndex>=defenderSearchBox.length){
			//TODO return random tile - returning for now
			return null;
		}
		currentDefenderIndex = nextDefenderIndex;
		nextDefenderIndex++;
		return defenderSearchBox[currentDefenderIndex];
	}
	
	public void setDefenderLocation(int tileX, int tileY){
		double xCoord = (30.48*tileX);
		double yCoord = (30.48*tileY);
		
		defenderSearchBox = new double[][]
				{
					/*{xCoord-30.48, yCoord-30.48},*/
					{xCoord, yCoord-(30.48+22)},
					/*{xCoord+30.48, yCoord-30.48},*/
					{xCoord+(30.48+22), yCoord},
					/*{xCoord+30.48, yCoord+30.48},*/
					{xCoord, yCoord+(30.48+22)},
					/*{xCoord-30.48, yCoord+30.48},*/
					{xCoord-(30.48+22), yCoord},
				};
	}
	
	public void markCurrentAttackerLocationBlocked(){
		currentAttackerIndex = nextAttackerIndex;
		nextAttackerIndex++;
	}
	
	public void markCurrentDefenderLocationAsBlocked(){
		//RConsole.println("Marking current x and y"+defenderSearchBox[currentDefenderIndex][0]+","+defenderSearchBox[currentDefenderIndex][1]);
		currentDefenderIndex = nextDefenderIndex;
		nextDefenderIndex++;
	}
	
	public double[] getNextXYCoordinate(double xCurrentCoord, double xDestCoord, double yCurrentCoord, double yDestCoord){
		double xNextCoord = xCurrentCoord;
		double yNextCoord = yCurrentCoord;
		
	
		if(nextToTravel == TRAVEL_IN_X){
			xNextCoord = getNextXCoord(xCurrentCoord, xDestCoord);
			nextToTravel = TRAVEL_IN_Y;
		}else{
			yNextCoord = getNextYCoord(yCurrentCoord, yDestCoord);
			nextToTravel = TRAVEL_IN_X;
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
