from controller import Supervisor
import math
import heapq

robot = Supervisor()

MAX_SPEED = 6.28
TURNING_SPEED = MAX_SPEED / 2
leftSpeed = MAX_SPEED
rightSpeed = MAX_SPEED

goalDirectionVariance = 1.0
goalPositionVariance = 0.03

timestep = int(robot.getBasicTimeStep())

states = ['forward', 'turning', 'stop', 'blocked']
current_state = states[1]

nodeNames = ['blueNode', 'greenNode', 'whiteNode', 'redNode', 'orangeNode', 'purpleNode', 'yellowNode']

GRAPH = {
    'blueNode': ['greenNode', 'whiteNode', 'purpleNode'],
    'greenNode': ['blueNode', 'whiteNode', 'redNode', 'yellowNode'],
    'whiteNode':['blueNode', 'greenNode'],
    'redNode': ['greenNode'],
    'orangeNode': ['purpleNode', 'yellowNode'],
    'purpleNode': ['blueNode', 'orangeNode'], 
    'yellowNode': ['greenNode', 'orangeNode']
}

goals = ['greenNode', 'whiteNode', 'redNode', 'yellowNode', 'orangeNode', 'purpleNode', 'blueNode']

robotNode = robot.getSelf()
nodeRefs = {name: robot.getFromDef(name) for name in nodeNames}

def get_node_position(name):
    t = nodeRefs[name].getField("translation").getSFVec3f()
    return t[0], t[1]

def heuristic(a, b):
    ax, ay = get_node_position(a)
    bx, by = get_node_position(b)
    return math.sqrt((bx - ax)**2 + (by - ay)**2)

def astar(start, goal):
    open_list = []
    heapq.heappush(open_list, (0.0, start))

    came_from = {}          
    g_score = {start: 0.0}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            
            while current in came_from:
                path.append(current)
                current = came_from[current]
                
            path.append(start)
            path.reverse()
            return path

        for neighbour in GRAPH.get(current, []):
            cx, cy = get_node_position(current)
            nx, ny = get_node_position(neighbour)
            edge_cost= math.sqrt((nx - cx)**2 + (ny - cy)**2)
            tentative_g =  g_score[current] + edge_cost

            if tentative_g < g_score.get(neighbour, float('inf')):
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                f =  tentative_g + heuristic(neighbour, goal)
                heapq.heappush(open_list, (f, neighbour))

    return []

def build_full_route(start, goal_list):
    full_route = [start]
    current = start

    
    for goal in goal_list:
        if current == goal:
            continue
        
        segment = astar(current, goal)
        
        if not segment:
            print(f"No path from fund from '{current}' to '{goal}'")
            break

        full_route.extend(segment[1:])
        current = goal
        
    return full_route

route = build_full_route('blueNode', goals)
routeStep = 0

if not route or len(route) < 2:
    print("No path")
    current_state = states[2]
    targetNodeName = 'blueNode'
else:
    routeStep = 1
    targetNodeName = route[routeStep]

currentNodeName = route[0]
destX,destY = get_node_position(targetNodeName)

gps = robot.getDevice('gps')
gps.enable(timestep)
gpsValues = []

ds_names = ["ds_left", "ds_right"]
sensors = {name: robot.getDevice(name) for name in ds_names}

for sensor in sensors.values():
    sensor.enable(timestep)

wheels = []
wheelsName = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for i in range(4):
    wheels.append(robot.getDevice(wheelsName[i]))
    wheels[i].setPosition(float('inf'))

for w in wheels:
    w.setVelocity(MAX_SPEED)

def getRotation(x, y, destX, destY):
    return math.atan2(destY - y, destX - x)

def turnRobot(x, y, destX, destY, currentRotation):
    targetRotation =getRotation(x, y, destX, destY)
    targetRotationDegrees = targetRotation * (180 / math.pi)
    currentRotationDeg = currentRotation * (180 / math.pi)

    global current_state

    if abs(currentRotationDeg - targetRotationDegrees) < goalDirectionVariance:
        current_state = states[0]
        return MAX_SPEED, MAX_SPEED

    if currentRotationDeg >= 0.0 and targetRotationDegrees >= 0.0:
        if currentRotationDeg > targetRotationDegrees:
            return TURNING_SPEED, -TURNING_SPEED
        else:
            return -TURNING_SPEED, TURNING_SPEED
        
    elif currentRotationDeg >= 0.0 and targetRotationDegrees <= 0.0:
        convertedTargetAngle = 360 + targetRotationDegrees
        if abs(currentRotationDeg - targetRotationDegrees) < goalPositionVariance:
            current_state = states[0]
            return MAX_SPEED, MAX_SPEED
        elif convertedTargetAngle - currentRotationDeg > 180:
            return TURNING_SPEED, -TURNING_SPEED
        else:
            return -TURNING_SPEED, TURNING_SPEED
        
    elif currentRotationDeg <= 0.0 and targetRotationDegrees <= 0.0:
        if currentRotationDeg > targetRotationDegrees:
            return TURNING_SPEED, -TURNING_SPEED
        else:
            return -TURNING_SPEED, TURNING_SPEED
        
    elif currentRotationDeg <= 0.0 and targetRotationDegrees >= 0.0:
        convertedRobotAngle = 360 + currentRotationDeg

        if convertedRobotAngle - targetRotationDegrees < 180:
            return TURNING_SPEED, -TURNING_SPEED
        else:
            return -TURNING_SPEED, TURNING_SPEED
        
    return 0, 0


while robot.step(timestep) != -1:
    print(current_state)
    print(f"Going to {targetNodeName} from {currentNodeName}")
    print('-------------------')

    rotationField = robotNode.getField("rotation")
    rotation = rotationField.getSFVec3f()[2] * rotationField.getSFVec3f()[3]

    gpsValues= gps.getValues()

    for i in range(3):
        if abs(gpsValues[i]) < 0.0001:
            gpsValues[i] = 0

    dx = abs(destX - gpsValues[0])
    dy = abs(destY -  gpsValues[1])

    if dx < goalPositionVariance and  dy < goalPositionVariance:
        print(f"Currently at {targetNodeName}")
        currentNodeName = targetNodeName
        routeStep += 1

        if routeStep >= len(route):
            print("Finished!!!")
            current_state = states[2]
        else:
            nextNodeName = route[routeStep]

            if nextNodeName not in GRAPH.get(currentNodeName, []):
                print(f"Obstacle between {currentNodeName} to   {nextNodeName}")
                current_state = states[3]
                
            else:
                print(f"Nxt is {nextNodeName}")
                targetNodeName = nextNodeName
                destX,  destY = get_node_position(targetNodeName)
                current_state = states[1]


    if current_state in ('stop', 'blocked'):
        leftSpeed = 0
        rightSpeed = 0
    elif current_state == 'forward':
        leftSpeed = MAX_SPEED
        rightSpeed = MAX_SPEED
    elif current_state == 'turning':
        leftSpeed, rightSpeed = turnRobot(
            gpsValues[0], gpsValues[1], destX, destY, rotation)

    wheels[0].setVelocity(leftSpeed)
    wheels[2].setVelocity(leftSpeed)
    wheels[1].setVelocity(rightSpeed)
    wheels[3].setVelocity(rightSpeed)