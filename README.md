# Webots GPS Path Planning with A\*
 
A Webots robotics simulation where a 4-wheeled robot autonomously navigates between colour-coded waypoint nodes using **A\* pathfinding** on a graph of allowed edges, driven by a GPS sensor and a Finite State Machine.
 
> SETU 2026 | Intelligent Cyber Physical System
> | Starter code provided by Dr Oisin Cawley
 
---
 
## Demo
 
The robot spawns at **blueNode**, plans an optimal route through all 7 nodes using A\*, and executes it hop-by-hop. It avoids blocked paths (nodes separated by wooden box obstacles) by routing through intermediate nodes automatically.
 
```
A* planned route: blueNode -> greenNode -> whiteNode -> greenNode -> redNode
                  -> greenNode -> yellowNode -> orangeNode -> purpleNode -> blueNode
```
 
---
 
## Features
 
- **A\* Pathfinding** — finds the shortest valid path between any two nodes using Euclidean distance as both edge cost and heuristic
- **Adjacency graph** — allowed connections stored as a plain `dict` of lists; blocked paths simply omitted
- **GPS navigation** — real-time position from Webots GPS sensor drives all movement decisions
- **Finite State Machine** — clean 4-state FSM: `turning → forward → stop / blocked`
- **Edge safety check** — robot halts with a console warning if a planned step has no valid edge
- **7 waypoint nodes** — original 4 (blue, green, white, red) plus 3 new (orange, purple, yellow)
 
---
 
## Project Structure
 
```
├── worlds/
│   └── myGPSDemo.wbt          # Webots world file (nodes, obstacles, robot)
├── controllers/
│   └── my_gps_controller/
│       └── my_gps_controller.py   # Main robot controller
└── README.md
```
 
---
 
## How It Works
 
### 1. Graph Definition
 
Connections between nodes are defined as an adjacency dict. Paths blocked by physical obstacles are simply not listed:
 
```python
GRAPH = {
    'blueNode':   ['greenNode', 'whiteNode', 'purpleNode'],
    'greenNode':  ['blueNode', 'whiteNode', 'redNode', 'yellowNode'],
    'whiteNode':  ['blueNode', 'greenNode'],
    'redNode':    ['greenNode'],
    'orangeNode': ['purpleNode', 'yellowNode'],
    'purpleNode': ['blueNode', 'orangeNode'],
    'yellowNode': ['greenNode', 'orangeNode'],
}
```
 
Notably absent: `blueNode ↔ redNode` and `whiteNode ↔ redNode` — both blocked by wooden box obstacles in the world.
 
### 2. A\* Algorithm
 
`astar(start, goal)` searches the graph using:
- **g-score**: cumulative Euclidean travel distance along the path so far
- **heuristic**: straight-line distance from current node to goal (admissible → guarantees optimal path)
- **open list**: Python `heapq` min-heap keyed on `f = g + h`
 
```python
def astar(start, goal):
    open_list = []
    heapq.heappush(open_list, (0.0, start))
    came_from = {}
    g_score   = {start: 0.0}
    ...
```
 
Route planning runs once at startup. `build_full_route()` chains A\* calls across all goal nodes and stitches the segments into a single waypoint list.
 
### 3. Finite State Machine
 
| State | Behaviour |
|---|---|
| `turning` | Rotates in place toward the next waypoint |
| `forward` | Drives straight at `MAX_SPEED` |
| `stop` | All wheels halted — route complete |
| `blocked` | All wheels halted — invalid edge detected |
 
Transitions are triggered by arrival detection (GPS within `goalPositionVariance = 0.03 m`) and heading alignment (within `goalDirectionVariance = 1.0°`).
 
---
 
## Setup & Running
 
### Requirements
 
- [Webots R2023b](https://cyberbotics.com/) or later
- Python 3.x (bundled with Webots)
- No external Python packages — only standard library (`math`, `heapq`)
 
### Steps
 
1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/webots-astar-path-planning.git
   ```
 
2. Open Webots and load the world:
   ```
   File → Open World → worlds/myGPSDemo.wbt
   ```
 
3. Ensure the controller is set to `my_gps_controller` in the Robot node properties.
 
4. Press **Play** — the robot will print the A\* planned route to the console and begin navigating.
 
---
 
## Extending the Project
 
**Add a new node:**
1. Add a `DEF myNode Solid { ... }` entry in `myGPSDemo.wbt` with the desired colour and position
2. Add `'myNode'` to the `nodeNames` list in the controller
3. Add its edges to `GRAPH`
4. Add it to the `goals` list — A\* handles the rest automatically
 
**Change the visit order:** just reorder the `goals` list. No route logic needs updating.
 
**Add an obstacle:** remove the corresponding edges from `GRAPH`. A\* will route around them.
