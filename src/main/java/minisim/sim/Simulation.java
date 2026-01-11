package minisim.sim;

public class Simulation 
{
    public static void main(String[] args) 
    {
    	long delay = 1000;
        Environment environment = new Environment("map3.txt", 100, 200);
        Robot robot = new Robot("Robby", environment);

        while (!robot.isAtTarget() && !environment.isSimulationOver()) 
        {
            robot.takeAction();
            environment.nextTurn();
            sleep(delay);
        }

        if (robot.isAtTarget()) 
        {
            System.out.println("The robot reached the target: "+robot);
        } 
        else 
        {
            System.out.println("Max turn reached, simulation ended.");
        }
    }
    
    public static void sleep(long delay)
    {
    	try
    	{
    		Thread.currentThread().sleep(delay);
    	}
    	catch(Exception e)
    	{
    		e.printStackTrace();
    	}
    }
}