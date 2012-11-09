
public class SearchAlgorithm {
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
	
	private int getNextXTile(int xCurrentTile, int destXTile){
		int nextXTile = destXTile;
		return nextXTile;
	}
	
	private int getNextYTile(int xCurrentTile, int destYTile){
		int nextYTile = destYTile;
		return nextYTile;
	}
}
