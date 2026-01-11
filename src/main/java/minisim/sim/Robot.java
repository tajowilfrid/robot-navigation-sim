package minisim.sim;

import java.util.Random;

import minisim.sim.Environment.Direction;
import minisim.sim.Environment.Field;

public class Robot 
{
	private static final Random random = new Random(); 
	
	private String name;
    private int x, y;
    private int energy;
    private Environment environment;
    private Direction lastdir;

    public Robot(String name, Environment environment) 
    {
    	this.name = name;
        this.environment = environment;
        environment.addRobot(this);
    }
    
    public String getName() 
    {
		return name;
	}

	public int getX() 
	{
		return x;
	}
	
	public void setX(int x) 
	{
		this.x = x;
	}

	public int getY() 
	{
		return y;
	}
	
	public void setY(int y) 
	{
		this.y = y;
	}

	public int getEnergy() 
	{
		return energy;
	}
	
	public void setEnergy(int energy) 
	{
		this.energy = energy;
	}

	public boolean isAtTarget() 
    {
        return environment.isTarget(x, y);
    }

    public void takeAction() 
    {
    	Field[][] env = environment.getLocalEnvironment(getName(), 0);
    	//System.out.println("Can see: "+x+" "+y);
    	//System.out.println(Environment.getMapAsString(env));
    	
    	Direction dir;
    	
    	if(env[0][0]==Field.CHARGER && getEnergy()<80)
    	{
    		dir = Direction.NONE;
    	}
    	else
    	{
	    	if(lastdir!=null && environment.canMoveTo(getName(), lastdir) && !shallExplore())
	    	{
	    		dir = lastdir;
	    	}
	    	else
	    	{
	    		do
	    		{
	    			dir = getRandomDirection();
	    			//System.out.println("dir: "+dir+" "+environment.canMoveTo(getName(), dir));
	    		}
	    		while(!environment.canMoveTo(getName(), dir));
	    	}
    	}
        
    	environment.moveRobot(getName(), dir);
    	
        lastdir = dir;
        
        System.out.println("Round: "+environment.getTurn()+" Robot moved to: (" + x + ", " + y + ")"+" "+dir);
    }
    
    public Direction getRandomDirection() 
    {
        Direction[] directions = Direction.values();
        return directions[random.nextInt(directions.length-1)];
    }
    
    public boolean shallExplore()
    {
    	return random.nextFloat()>0.7;
    }

	@Override
	public String toString() 
	{
		return "Robot [name=" + name + "]";
	}
}