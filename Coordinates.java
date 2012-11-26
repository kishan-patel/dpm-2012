public class Coordinates{
	public double x;
	public double y;
	public double theta;
	public Coordinates()
	{
		x=0;
		y=0;
		theta=0;
	}
	public Coordinates(double x, double y, double theta)
	{
		this.x=x;
		this.y=y;
		this.theta=theta;
	}
	
	public void copy(Coordinates p)
	{
		x=p.x;
		y=p.y;
		theta=p.theta;
		
	}
}
