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

    // Memory for found charging stations
    private List<int[]> knownChargers;

    // Configuration for Task 3
    private static final int ENERGY_THRESHOLD = 40; // If energy < 40, go charge
    private static final double COST_NORMAL = 1.0;
    private static final double COST_LAVA = 100.0; // High penalty for Lava

    public Robot(String name, Environment environment) 
    {
        this.name = name;
        this.environment = environment;
        environment.addRobot(this);
        this.currentPath = new LinkedList<>();
        this.knownChargers = new ArrayList<>();
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

        // If on a charger and not full, wait!
        // check the own coordinates in the internal map
        if (internalMap[y][x] == Field.CHARGER && energy < 100) {
            // Sending NONE triggers the recharge effect in Environment without moving
            environment.moveRobot(name, Direction.NONE);
            System.out.println("Round: " + environment.getTurn() + " |Robot is charging... (Energy: " + energy + "%)");
            return; // Skip the rest of the turn
        }

        // To determine current destination
        int[] finalTarget = environment.getGoal();
        String targetDescription = "Target";

        // If energy is critical and we know a charger, go there, but only if we are not already full
        if (this.energy < ENERGY_THRESHOLD && !knownChargers.isEmpty()) {
            int[] nearestCharger = getNearestKnownCharger();
            if (nearestCharger != null) {
                finalTarget = nearestCharger;
                targetDescription = "Charger (" + finalTarget[0] + "," + finalTarget[1] + ")";
            } else {
                System.out.println("Low Energy! No reachable charger found. Risking it to Target.");
            }
        }

        // Recalculate if path is empty, blocked, or if we switched targets
        if (currentPath.isEmpty() || isPathBlocked()) {
            // We clear the path to force A* to find the best route to the current target
            System.out.println("Recalculating path to " + targetDescription + " | Energy: " + energy);
            calculatePathWithMemory(finalTarget[0], finalTarget[1]);
        }
        
        // execution of next step in path
        if (currentPath.isEmpty()) {
            System.out.println("No path found (or already at destination).");
            return;
        }

        Direction nextDir = currentPath.poll();

        if (nextDir != null) {
            if (environment.canMoveTo(name, nextDir)) {
                environment.moveRobot(name, nextDir);
                System.out.println("Round: " + environment.getTurn() + " | Robot moved to (" + x + ", " + y + ") via " + nextDir + " | Energy: " + energy);
            } else {
                System.out.println("Unexpected blockage at (" + x + "," + y + ")! Recalculating next turn...");
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
                    
                    Field seenField = view[dy][dx];
                    // Save what we see into the memory
                    internalMap[globalY][globalX] = seenField;

                    // NEW: If we see a charger, add it to the list
                    if (seenField == Field.CHARGER) {
                        addKnownCharger(globalX, globalY);
                    }
                }
            }
        }
    }

    /**
     * Helper to add a charger to the list only if it's new.
     */
    private void addKnownCharger(int cx, int cy) {
        for (int[] c : knownChargers) {
            if (c[0] == cx && c[1] == cy) return; // Already known
        }
        knownChargers.add(new int[]{cx, cy});
        System.out.println("Discovered new Charger at (" + cx + "," + cy + ")");
    }

    /**
     * Scans for the closest Charger (C) based on simple distance.
     */
    private int[] getNearestKnownCharger() {
        int[] bestCharger = null;
        double minDist = Double.MAX_VALUE;

        for (int[] c : knownChargers) {
            double dist = calculateHeuristic(this.x, this.y, c[0], c[1]);
            if (dist < minDist) {
                minDist = dist;
                bestCharger = c;
            }
        }
        return bestCharger;
    }

    /**
     * Checks if the next step is a hard OBSTACLE.
     * Lava is NOT considered a block here, because A* decides if we walk on it.
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

        if (checkY >= 0 && checkY < internalMap.length && checkX >= 0 && checkX < internalMap[0].length) {
            Field f = internalMap[checkY][checkX];
            // Only strictly block Obstacles. 
            if (f == Field.OBSTACLE) {
                return true;
            }
        }
        return false;
    }

    /**
     * * Runs A* algorithm on the INTERNAL MAP to find a path to the target
     * Weighted A* Implementation.
     * Calculates path to (targetX, targetY).
     */
    private void calculatePathWithMemory(int targetX, int targetY) {
        // Clear old path
        currentPath.clear();

        
        // Priority Queue ordered by F cost (lowest first)
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        // Set to keep track of visited coordinates
        Set<String> closedSet = new HashSet<>();
        
         // Initialize start node
        Node startNode = new Node(this.x, this.y, null, 0, calculateHeuristic(this.x, this.y, targetX, targetY));
        openSet.add(startNode);
        
        while (!openSet.isEmpty()) {
            // Get node with lowest F cost
            Node current = openSet.poll();
            
             // Check if we reached the target
            if (current.x == targetX && current.y == targetY) {
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
                    
                    if (knownField == Field.OBSTACLE) continue; // Hard wall

                    double moveCost = COST_NORMAL;
                    
                    // If it is Lava, increase cost significantly to avoid it if possible
                    if (knownField == Field.LAVA) {
                        moveCost = COST_LAVA; 
                    }
                    
                    double newGCost = current.gCost + moveCost;
                    double newHCost = calculateHeuristic(nextX, nextY, targetX, targetY);
                    
                    // Add to open set
                    Node neighbor = new Node(nextX, nextY, current, newGCost, newHCost);
                    neighbor.directionFromParent = dir; // Store direction directly
                    openSet.add(neighbor);
                }
            }
        }
        // If no path found, output info
        System.out.println("A* could not find a path to (" + targetX + "," + targetY + ")");
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
