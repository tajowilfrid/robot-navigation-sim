package minisim.sim;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Environment 
{
	enum Direction 
	{
		LEFT, RIGHT, UP, DOWN, NONE
	}
	
	public enum Field
	{
		EMPTY("."),
	    OBSTACLE("#"),
	    LAVA("L"),
	    CHARGER("C"), // "\u26A1"
	    START("S"),
	    TARGET("T");
	
	    private final String symbol;
	
	    Field(String symbol) 
	    {
	        this.symbol = symbol;
	    }
	
	    public String getSymbol() 
	    {
	        return symbol;
	    }
	    
	    @Override
	    public String toString() 
	    {
	    	return getSymbol();
	    }
	    
	    public static Field fromSymbol(String symbol) 
	    {
            for (Field field : Field.values()) 
            {
                if (field.getSymbol().equals(symbol)) 
                    return field;
            }
            return null;
        }
	}
	
    private Field[][] grid; 
    private int[] start;
    private int[] target;
    private int initialEnergy;
    private int turn = 0;
    private int maxTurns;
    private MapGui gui;
    private Map<String, Robot> robots = new HashMap<>();

    public Environment(String filename, int initialEnergy, int maxTurns) 
    {
    	this.initialEnergy = initialEnergy;
    	this.grid = loadMap(filename);
    	System.out.println(getMapAsString(this.grid));
        this.maxTurns = maxTurns;
        this.gui = new MapGui(this);
        this.gui.setVisible(true);
    }
    
    public Map<String, Robot> getRobots() 
    {
		return robots;
	}

	public Field[][] loadMap(String filename)
    {
        List<String> lines = new ArrayList<>();
        
        try (
        	InputStream inputStream = getClass().getClassLoader().getResourceAsStream(filename);
        	BufferedReader reader = new BufferedReader(new InputStreamReader(inputStream))
        ) 
        {
        	String line;
        	while ((line = reader.readLine()) != null) 
        	{
        		lines.add(line.replace(" ", ""));
            }
        } 
        catch (IOException e) 
        {
        	e.printStackTrace();
        }

        int height = lines.size();
        int width = lines.get(0).length();
        Field[][] map = new Field[height][width];
        
        for (int y = 0; y < height; y++) 
        {
            for (int x = 0; x < width; x++) 
            {
                char currentChar = lines.get(y).charAt(x);
                Field field = Field.fromSymbol(String.valueOf(currentChar));
                
                if (field != null) {
                    map[y][x] = field;
                    
                    if (field == Field.START) 
                    {
                        start = new int[]{x, y};
                    }
                    
                    if (field == Field.TARGET) 
                    {
                        target = new int[]{x, y};
                    }
                }
            }
        }
        return map;
    }
    
    public static String getMapAsString(Field[][] grid) 
    {
        StringBuilder mapString = new StringBuilder();

        for (int y = 0; y < grid.length; y++) 
        {
            for (int x = 0; x < grid[y].length; x++) 
            {
                mapString.append(grid[y][x].toString());
            }
            mapString.append("\n"); 
        }
        return mapString.toString();
    }
    
    public void addRobot(Robot robot)
    {
    	robots.put(robot.getName(), robot);
    	
        int[] start = getStart();
        robot.setX(start[0]);
        robot.setY(start[1]);
        robot.setEnergy(getInitialEnergy());
    }
    
    public Field[][] getEnvironment()
    {
    	return grid;
    }
    
    public Field[][] getLocalEnvironment(String name)
    {
    	return getLocalEnvironment(name, 2);
    }
    
    public Field[][] getLocalEnvironment(String name, int radius) 
    {
    	Robot ri = getRobot(name);
    	
        int envSize = 2 * radius + 1;
        Field[][] ret = new Field[envSize][envSize];

        for (int dy = -radius; dy <= radius; dy++) 
        {
            for (int dx = -radius; dx <= radius; dx++) 
            {
                int nx = ri.getX() + dx;
                int ny = ri.getY() + dy;

                if (nx >= 0 && ny >= 0 && nx < grid[0].length && ny < grid.length)
                {
                    ret[dy + radius][dx + radius] = grid[ny][nx];
                } 
                else 
                {
                    ret[dy + radius][dx + radius] = Field.OBSTACLE; 
                }
            }
        }
        return ret;
    }

    public boolean canMoveTo(String name, Direction dir) 
    {
    	Robot r = getRobot(name);
    	
    	if(dir==Direction.NONE)
    		return true;
    	else if(dir==Direction.RIGHT)
    		return canMoveTo(r.getX()+1, r.getY());
    	else if(dir==Direction.LEFT)
    		return canMoveTo(r.getX()-1,r.getY());
    	else if(dir==Direction.UP)
    		return canMoveTo(r.getX(),r.getY()-1);
    	else if(dir==Direction.DOWN)
    		return canMoveTo(r.getX(),r.getY()+1);
    	throw new IllegalArgumentException();
    }
    
    public boolean canMoveTo(int x, int y) 
    {
    	//System.out.println(x+" "+y+" "+grid[y][x]+" "+(grid[y][x] != Field.OBSTACLE));
        return x >= 0 && y >= 0 && y < grid.length && x < grid[0].length && grid[y][x] != Field.OBSTACLE; 
    }

    public void moveRobot(String name, Direction dir) 
    {
    	//System.out.println("Env move: "+name+" "+dir);
    	
    	Robot r = getRobot(name);
    	
    	if(!canMoveTo(name, dir))
    		throw new RuntimeException("Invalid move: "+dir);
    	
    	if(grid[r.getY()][r.getX()]==Field.CHARGER)
    		r.setEnergy(Math.min(100, r.getEnergy()+20));
    	else if(grid[r.getY()][r.getX()]==Field.LAVA)
    		r.setEnergy(Math.max(0, r.getEnergy()-20));
    	
    	boolean moved = true;
    	
    	if(r.getEnergy()>0)
    	{
	    	if(dir==Direction.RIGHT)
	    		r.setX(r.getX()+1);
	    	else if(dir==Direction.LEFT)
	    		r.setX(r.getX()-1);
	    	else if(dir==Direction.UP)
	    		r.setY(r.getY()-1);
	    	else if(dir==Direction.DOWN)
	    		r.setY(r.getY()+1);
	    	else
	    		moved = false;
	    	
	    	if(moved)
	    		r.setEnergy(r.getEnergy()-1);
    	}
    	else
    	{
    		throw new RuntimeException("No energy");
    	}
    	
        gui.repaint();  
    }
    
    public boolean isTarget(int x, int y) 
    {
        return x == target[0] && y == target[1];
    }

    public int[] getStart() 
    {
        return start;
    }

    public int[] getGoal() 
    {
        return target;
    }
    
    public int getInitialEnergy() 
    {
        return initialEnergy;
    }

    public void nextTurn() 
    {
        turn++;
    }
    
    public int getTurn() 
    {
		return turn;
	}

	public boolean isSimulationOver() 
    {
        return turn >= maxTurns;
    }
	
	public Robot getRobot(String name)
	{
		Robot ri = robots.get(name);
    	
    	if(ri==null)
    		throw new RuntimeException("Robot unknown: "+name);
    	
    	return ri;
	}
}