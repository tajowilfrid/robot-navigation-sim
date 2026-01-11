# Robot Simulation (Minisim)

This project implements an intelligent autonomous robot navigating a virtual grid world. It was developed as part of the "Verhaltenssteuerung" exercise. The robot evolves from a simple explorer to a state-based intelligent agent capable of energy management and path planning.

## Project Features (Tasks)

The solution covers all assignments:

### Task 1: Global Navigation
- **Algorithm:** A* (A-Star) Pathfinding.
- **Function:** Finds the shortest path from Start (S) to Target (T) assuming full knowledge of the map.

### Task 2: Local Navigation (Mapping)
- **Fog of War:** The robot starts with no map knowledge.
- **Sensors:** It explores the environment with a vision radius of 3 fields.
- **Internal Map:** Builds a memory representation (`internalMap`) of the world dynamically.
- **Re-planning:** Automatically detects blockages and recalculates paths using A* based on discovered obstacles.

### Task 3: Energy Management
- **Battery System:** The robot consumes energy for movement.
- **Weighted A*:** The pathfinding algorithm was enhanced to consider costs:
  - **Normal Step:** Cost 1.
  - **Lava (L):** Cost 100 (Avoids lava unless it's the only path).
  - **Walls (#):** Impassable.
- **Charging Logic:** If energy drops below 40%, the robot diverts to the nearest known Charger (C).

### Task 4: Behavior Model (Finite State Machine)
- The logic is structured using a robust **FSM**.
- **States:**
  - `SCANNING`: Updates internal map and sensors.
  - `ANALYZING`: Decides high-level goals (Target vs. Charger).
  - `PLANNING`: Calculates paths based on the current goal.
  - `MOVING`: Executes movement steps.
  - `CHARGING`: Waits at a station until the battery is full.
  - `FINISHED`: Stops simulation upon reaching the target.

---

## Setup & Installation

### Prerequisites
- Java JDK 11 or higher (Project uses Gradle 8.11).
- An IDE like IntelliJ IDEA or Eclipse.

### Build the Project
Open the terminal in the project root:

```bash
# Linux/Mac
./gradlew build

# Windows
gradlew.bat build

```

---

## How to Run & Test

The main entry point is `minisim.sim.Simulation`.

### Running the Simulation

1. Open `src/main/java/minisim/sim/Simulation.java`.
2. Run the `main` method.
3. The GUI window will open, showing the robot's movement.

### Test Scenarios

To verify the specific tasks, change the map file in `Simulation.java`:
`Environment environment = new Environment("mapX.txt", ...);`

#### Scenario A: Basic Navigation (Task 1 & 4)

* **Map:** `map1.txt` or `map3.txt`
* **Behavior:** The robot should efficiently navigate around walls directly to the target `T`.
* **FSM State:** Predominantly `MOVING` and `SCANNING`.

#### Scenario B: Energy Crisis (Task 3 & 4)

* **Map:** `map2.txt`
* **Behavior:**
1. Robot moves towards `T`.
2. When `Energy < 40`, console shows transition to `SEEK_CHARGER` (Target changes to Charger coords).
3. Robot navigates to `C`.
4. FSM State becomes `CHARGING` (Robot waits).
5. Once full (100%), Robot resumes journey to `T`.



#### Scenario C: Danger Avoidance (Task 3)

* **Map:** `map5.txt` (or custom map with Lava blocking the direct path)
* **Behavior:** The robot will take a longer path around Lava (`L`) because the cost (100) is higher than walking around. It only steps on Lava if no other path exists.

---

## Project Structure

* **Robot.java:** Core logic containing the FSM, A* algorithm, and memory management.
* **Simulation.java:** Setup of the Environment and Loop.
* **Environment.java:** Handles the grid, map loading, and physics.
* **MapGui.java:** Swing-based visualization.