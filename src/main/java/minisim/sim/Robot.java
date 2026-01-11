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
    
    // Internal memory of the world
    private Field[][] internalMap;
    private boolean isMapInitialized = false;

    // Stores the current calculated path
    private Queue<Direction> currentPath;

    public Robot(String name, Environment environment) 
    {
        this.name = name;
        this.environment = environment;
        environment.addRobot(this);
        this.currentPath = new LinkedList<>();
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
     * Initializes the internal map dimensions
     */
    private void initMemory() {
        Field[][] trueGrid = environment.getEnvironment();
        this.internalMap = new Field[trueGrid.length][trueGrid[0].length];
        this.isMapInitialized = true;
    }

    /**
     * Main simulation step for Local Navigation.
     * - Update internal map based on local view.
     * - Plan a path using A* on the internal map.
     * - Move.
     */
    public void takeAction() 
    {
        if (!isMapInitialized) {
            initMemory();
        }

        // explorae internal map with local view (Radius 3)
        updateInternalMap(3);

        // plan if we have no path or the current path is blocked by a newly discovered obstacle
        if (currentPath.isEmpty() || isPathBlocked()) {
            System.out.println("Recalculating path based on new knowledge...");
            calculatePathWithMemory();
        }

        // execution of next step in path
        Direction nextDir = currentPath.poll();

        if (nextDir != null) {
            if (environment.canMoveTo(name, nextDir)) {
                environment.moveRobot(name, nextDir);
                System.out.println("Round: " + environment.getTurn() + " | Robot moved to (" + x + ", " + y + ") via " + nextDir);
            } else {
                System.out.println("Unexpected blockage at (" + x + "," + y + ")! Waiting...");
                // Force recalculation next turn
                currentPath.clear(); 
            }
        } else {
            System.out.println("No path found or target reached.");
        }
    }

    /**
     * Looks around (radius) and saves the info into internalMap.
     */
    private void updateInternalMap(int radius) {
        // Get local view from environment
        Field[][] view = environment.getLocalEnvironment(name, radius);
        
        // Map local view coordinates to global internalMap coordinates
        for (int dy = 0; dy < view.length; dy++) {
            for (int dx = 0; dx < view[dy].length; dx++) {
                int globalY = this.y - radius + dy;
                int globalX = this.x - radius + dx;

                // Check boundaries of the map
                if (globalY >= 0 && globalY < internalMap.length && 
                    globalX >= 0 && globalX < internalMap[0].length) {
                    
                    // Save what we see into our memory
                    internalMap[globalY][globalX] = view[dy][dx];
                }
            }
        }
    }

    /**
     * Checks if the next step in our current queue is now known to be an obstacle.
     */
    private boolean isPathBlocked() {
        if (currentPath.isEmpty()) return false;
        
        Direction next = currentPath.peek();
        int checkX = x;
        int checkY = y;
        
        switch (next) {
            case UP: checkY--; break;
            case DOWN: checkY++; break;
            case LEFT: checkX--; break;
            case RIGHT: checkX++; break;
            default: break;
        }

        // If we know this spot and it is an obstacle or lava, the path is blocked/dangerous
        if (checkY >= 0 && checkY < internalMap.length && checkX >= 0 && checkX < internalMap[0].length) {
            Field f = internalMap[checkY][checkX];
            // If f is null, we don't know it yet, so we assume it's fine.
            if (f == Field.OBSTACLE || f == Field.LAVA) {
                return true;
            }
        }
        return false;
    }

    /**
     * Runs A* algorithm on the INTERNAL MAP to find a path to the target
     */
    private void calculatePathWithMemory() {
        // Clear old path
        currentPath.clear();
        
        int[] targetPos = environment.getGoal();
        
        // Priority Queue ordered by F cost (lowest first)
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        // Set to keep track of visited coordinates
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
                
                int nextX = current.x;
                int nextY = current.y;
                
                // Determine coordinates of the neighbor
                switch (dir) {
                    case UP: nextY--; break;
                    case DOWN: nextY++; break;
                    case LEFT: nextX--; break;
                    case RIGHT: nextX++; break;
                }
                
                // Boundary check
                if (nextY >= 0 && nextY < internalMap.length && nextX >= 0 && nextX < internalMap[0].length) {
                    
                    // Skip if already visited
                    if (closedSet.contains(nextX + "," + nextY)) continue;

                    // walkability check
                    Field knownField = internalMap[nextY][nextX];
                    
                    // If we know it's an OBSTACLE or LAVA, we ignore this path
                    if (knownField == Field.OBSTACLE || knownField == Field.LAVA) {
                        continue;
                    }
                    
                    double newGCost = current.gCost + 1;
                    double newHCost = calculateHeuristic(nextX, nextY, targetPos[0], targetPos[1]);
                    
                    // Add to open set
                    Node neighbor = new Node(nextX, nextY, current, newGCost, newHCost);
                    neighbor.directionFromParent = dir; // Store direction directly
                    openSet.add(neighbor);
                }
            }
        }
        System.out.println("Target unreachable in internal map! (Or stuck)");
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
    private void reconstructPath(Node endNode) {
        LinkedList<Direction> pathStack = new LinkedList<>();
        Node current = endNode;
        
        while (current.parent != null) {
            // To store the direction in the node, so we can just grab it
            if (current.directionFromParent != null) {
                pathStack.addFirst(current.directionFromParent);
            }
            current = current.parent;
        }
        this.currentPath.addAll(pathStack);
    }

    /**
     * Helper class to represent a position in the pathfinding process
     */
    private class Node implements Comparable<Node> {
        int x, y;
        Node parent; // Reference to previous node to retrace path
        double gCost; // Cost from start
        double hCost; // Heuristic cost to target
        double fCost; // Total cost (G + H)
        Direction directionFromParent; // Which move brought us here?

        public Node(int x, int y, Node parent, double gCost, double hCost) {
            this.x = x;
            this.y = y;
            this.parent = parent;
            this.gCost = gCost;
            this.hCost = hCost;
            this.fCost = gCost + hCost;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.fCost, other.fCost);
        }
    }

    @Override
    public String toString() 
    {
        return "Robot [name=" + name + "]";
    }
}
