package minisim.sim;

import java.util.*;

import minisim.sim.Environment.Direction;
import minisim.sim.Environment.Field;

public class Robot 
{
    private String name;
    private int x, y;
    private int energy;
    private Environment environment;
    
    // Stores the calculated path (list of directions to follow)
    private Queue<Direction> calculatedPath;

    public Robot(String name, Environment environment) 
    {
        this.name = name;
        this.environment = environment;
        environment.addRobot(this);
        this.calculatedPath = new LinkedList<>();
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

    /**
     * Executes the next step of the simulation.
     * Uses A* to calculate the path once, then follows it.
     */
    public void takeAction() 
    {
        // Calculate path if we haven't done it yet or if the path is empty
        if (calculatedPath.isEmpty()) {
            System.out.println("Calculating path using A*...");
            calculateAStarPath();
        }

        // Get the next step from our queue
        Direction nextDir = calculatedPath.poll();

        if (nextDir != null) {
            if (environment.canMoveTo(name, nextDir)) {
                environment.moveRobot(name, nextDir);
                System.out.println("Round: " + environment.getTurn() + " | Robot moved: " + nextDir);
            } else {
                System.out.println("Path blocked! Waiting...");
                // In a dynamic world, we would trigger a recalculation here
            }
        } else {
            System.out.println("No path found or target reached.");
        }
    }

    /**
     * Implementation of the A* (A-Star) Pathfinding Algorithm.
     * It populates the 'calculatedPath' queue with directions.
     */
    private void calculateAStarPath() {
        // Get the global map from the environment [cite: 221]
        Field[][] grid = environment.getEnvironment();
        int[] targetPos = environment.getGoal();
        
        // Priority Queue to store nodes to be explored, ordered by F cost
        PriorityQueue<Node> openSet = new PriorityQueue<>(Comparator.comparingDouble(n -> n.fCost));
        
        // Set to keep track of visited nodes to avoid cycles and redundant processing
        Set<String> closedSet = new HashSet<>();
        
        // Initialize start node
        Node startNode = new Node(this.x, this.y, null, 0, calculateHeuristic(this.x, this.y, targetPos[0], targetPos[1]));
        openSet.add(startNode);
        
        while (!openSet.isEmpty()) {
            // Get node with lowest F cost
            Node current = openSet.poll();
            
            // Check if we reached the target
            if (current.x == targetPos[0] && current.y == targetPos[1]) {
                reconstructPath(current);
                return;
            }
            
            // Mark current node as visited
            closedSet.add(current.x + "," + current.y);
            
            // Check all 4 neighbors (Up, Down, Left, Right)
            for (Direction dir : Direction.values()) {
                if (dir == Direction.NONE) continue;
                
                int newX = current.x;
                int newY = current.y;
                
                // Determine coordinates of the neighbor
                switch (dir) {
                    case UP: newY--; break;
                    case DOWN: newY++; break;
                    case LEFT: newX--; break;
                    case RIGHT: newX++; break;
                }
                
                // Check if the neighbor is valid
                if (newY >= 0 && newY < grid.length && newX >= 0 && newX < grid[0].length) {
                    
                    // Skip Obstacles
                    if (grid[newY][newX] == Field.OBSTACLE) continue;
                    
                    // Skip if already evaluated
                    if (closedSet.contains(newX + "," + newY)) continue;
                    
                    // Calculate costs
                    double newGCost = current.gCost + 1; // Distance from start (+1 per step)
                    double newHCost = calculateHeuristic(newX, newY, targetPos[0], targetPos[1]);
                    
                    // Add to open set
                    Node neighbor = new Node(newX, newY, current, newGCost, newHCost);
                    openSet.add(neighbor);
                }
            }
        }
        System.out.println("No path found!");
    }

    /**
     * Calculates the Manhattan distance heuristic.
     * Suitable for grid movements (4 directions).
     */
    private double calculateHeuristic(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    /**
     * Backtracks from the target node to the start node to build the list of directions
     */
    private void reconstructPath(Node targetNode) {
        LinkedList<Direction> pathStack = new LinkedList<>();
        Node current = targetNode;
        
        while (current.parent != null) {
            Node parent = current.parent;
            int dx = current.x - parent.x;
            int dy = current.y - parent.y;
            
            if (dx == 1) pathStack.addFirst(Direction.RIGHT);
            else if (dx == -1) pathStack.addFirst(Direction.LEFT);
            else if (dy == 1) pathStack.addFirst(Direction.DOWN);
            else if (dy == -1) pathStack.addFirst(Direction.UP);
            
            current = parent;
        }
        
        this.calculatedPath.addAll(pathStack);
        System.out.println("Path calculated with " + calculatedPath.size() + " steps.");
    }

    /**
     * Helper class to represent a position in the pathfinding process
     */
    private class Node {
        int x, y;
        Node parent; // Reference to previous node to retrace path
        double gCost; // Cost from start
        double hCost; // Heuristic cost to target
        double fCost; // Total cost (G + H)

        public Node(int x, int y, Node parent, double gCost, double hCost) {
            this.x = x;
            this.y = y;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }
    }

    @Override
    public String toString() 
    {
        return "Robot [name=" + name + "]";
    }
}