"""Boids Supervisor."""

from controller import *
import math

PI = math.pi

NB_ROBOTS = 9
FOV = 0.2
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

coord = (0, 0)

robot_node = [[0]*2 for i in range(NB_ROBOTS)]
orientationMatrix = [[0]*NB_ROBOTS for i in range(NB_ROBOTS)]
distMatrix = [[0]*NB_ROBOTS for i in range(NB_ROBOTS)]
trans_field = []
rota_field = []
emitter = robot.getEmitter("emitter")

### Initialization. Getting each robots and their postion and rotation field
for i in range(NB_ROBOTS):
    node = robot.getFromDef("EP"+str(i))
    tf = node.getField("translation")
    rf = node.getField("rotation")
    trans_field.append(tf)
    rota_field.append(rf)

### Get the coordinates from the translation field
def getCoordinates(trans_field):
    values = trans_field.getSFVec3f()
    coord = (values[0], values[2])
    return coord

### Get rotation from the rotation field
def getRotation(rota_field):
    values = rota_field.getSFRotation()
    rota = values[3]
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
    center = (f'{(x/total):.5f}', f'{(y/total):.5f}')
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

### Calculate the difference in rotation between the target rotation and the current one
def rotationDiff(current, target):
    if target == 0:
        return f'{target:.5f}'
    delta = target - current
    #get the shortest rotation to reach the target
    if abs(delta) > PI:
        if current > target:
            delta = (2*PI + target) - current
        else:
            delta = target - (2*PI + current)
    return f'{delta:.5f}'

### Counter value for the execution
counter = 0

while (robot.step(timestep) != -1):
    if counter % 5 == 0:
        for i in range(NB_ROBOTS):
            robot_node[i][0] = getCoordinates(trans_field[i])
            robot_node[i][1] = getRotation(rota_field[i])

        for i in range(NB_ROBOTS):
            for j in range(NB_ROBOTS):
                # register rotation angle of robots in range and is not itself
                if ((getDist(*robot_node[i][0], *robot_node[j][0]) <= FOV) and i != j):
                    orientationMatrix[i][j] = robot_node[j][1]
                    distMatrix[i][j] = robot_node[j][0]
                else:
                    orientationMatrix[i][j] = 999
                    distMatrix[i][j] = 999
        ### send information to each robots
        for i in range(NB_ROBOTS):
            r = "t" + rotationDiff(transform(robot_node[i][1]), transform(avgRotation(orientationMatrix[i])))
            packet = [r]
            emitter.setChannel(i + 1)
            for item in packet:
                msg = bytes(item, 'utf-8')
                emitter.send(msg)
                counter += 1
    else:
        counter += 1
        if counter > 1000:
            counter = 0
