"""Boids Supervisor."""

from controller import *
import math
import random

PI = math.pi

### Set to the exact number of robots used
NB_ROBOTS = 9
### Controls the "Field of View" of the robots
FOV = 0.2

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

### Initialization of the different variables / matrices used to store the information
robot_node = [[0]*2 for i in range(NB_ROBOTS)]
orientationMatrix = [[0]*NB_ROBOTS for i in range(NB_ROBOTS)]
distMatrix = [[0]*NB_ROBOTS for i in range(NB_ROBOTS)]
trans_field = []
rota_field = []
coord = (0, 0)
emitter = robot.getEmitter("emitter")

### Set the rotation to a random value in range (-PI/2, PI/2) at launch
def setRotation(rota_field):
    values = rota_field.getSFRotation()
    orientation = random.uniform(-PI/2, PI/2)
    set = [values[0], values[1], values[2], orientation]
    rota_field.setSFRotation(set)
    return

### Get the coordinates from the translation field
def getCoordinates(trans_field):
    values = trans_field.getSFVec3f()
    coord = (values[0], values[2])              # get the relevant values for the X and Z axis, store them in a tuple
    return coord

### Get rotation from the rotation field
def getRotation(rota_field):
    values = rota_field.getSFRotation()
    rota = values[3]                            # get the relevant values for the rotation around the Z axis
    return rota

### Calulculate the average rotation from a list of rotation
# Don't use transform() before using this function
def avgRotation(*rotaList):
    rotation = 0
    total = 0
    for i in rotaList:
        for r in i:
            if (r != 999):
                rotation += r
                total += 1
    if total == 0 :
        total = 1
    return rotation / total

### Calculate the coordinate of the average point from a list of coordinates
def centerMass(*distList):
    x = 0
    y = 0
    total = 0
    for i in distList:
        for d in i:
            if (d != 999):
                x += d[0]
                y += d[1]
                total += 1
    if total == 0 :
        total = 1
    center = ((x/total), (y/total))
    return center

### Calculate the distance from point(x1,y1) to point(x2,y2)
def getDist(x1, y1, x2, y2):
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return dist

#transform [0;pi][0;-pi] range to [0,2pi]
def transform(rotation):
    if rotation < 0:
        rotation = 2*PI + rotation
    return rotation

### Calculate the angle between the robot and the center of the detected group
def centerMassdiff(center_x,center_y,robot_x, robot_y):
    delta = (math.atan2((robot_y - center_y),(robot_x-center_x)))
    return f'{delta:.5f}'

### Calculate the difference in rotation between the target rotation and the current one
def rotationDiff(current, target):
    if target == 0:
        return f'{target:.5f}'
    delta = current - target
    #get the shortest rotation to reach the target (if > PI go the other way around)
    if abs(delta) > PI:
        if current < target:
            delta = (2*PI + current) - target
        else:
            delta = current - (2*PI + target)
    return f'{delta:.5f}'


### Initialization. Getting a handle of each robots and their translation and rotation field
### Only done once, after launching
for i in range(NB_ROBOTS):
    node = robot.getFromDef("EP"+str(i))        # get the handle of a robot from their DEF name
    tf = node.getField("translation")           # get the handle on the translation field
    rf = node.getField("rotation")              # get the handle on the translation field
    setRotation(rf)                             # set the rotation to a random value
    trans_field.append(tf)                      # Add the access to the translation field to the appropriate array
    rota_field.append(rf)                       # Add the access to the translation field to the appropriate array

### Counter value for the execution
counter = 0

while (robot.step(timestep) != -1):
    if counter % 5 == 0:                        # Only do the calculation and send the info once in a while. For performance and stop backlog at robots
        # get the coordinates and rotation of each robot
        for i in range(NB_ROBOTS):
            robot_node[i][0] = getCoordinates(trans_field[i])
            robot_node[i][1] = getRotation(rota_field[i])
        # compare each robots to the others
        for i in range(NB_ROBOTS):
            for j in range(NB_ROBOTS):
                # register rotation angle of robots in FOV and is not itself
                if ((getDist(*robot_node[i][0], *robot_node[j][0]) <= FOV) and i != j):
                    #matrices for orientation and coordinates of robots in range of robot i
                    orientationMatrix[i][j] = robot_node[j][1]
                    distMatrix[i][j] = robot_node[j][0]
                else:
                    # if the robot j is not in range if robot i, put a specific value that can be discarded later
                    orientationMatrix[i][j] = 999
                    distMatrix[i][j] = 999
        ### send information to each robots
        for i in range(NB_ROBOTS):
            # Calculate the average rotation and the angle to the center of mass of the groupe
            r = "t" + rotationDiff(transform(robot_node[i][1]), transform(avgRotation(orientationMatrix[i])))
            c = "c" + centerMassdiff(*centerMass(distMatrix[i]),*robot_node[i][0])
            packet = [r,c]
            # set emitter channel to the robot i channel
            emitter.setChannel(i + 1)
            # encryte each information to send, to be read by the receiver
            for item in packet:
                msg = bytes(item, 'utf-8')
                emitter.send(msg)
        counter += 1
    else:
        counter += 1
    if counter > 1000:
        counter = 0
