"""Boids Supervisor."""

from controller import *
import math
import struct

PI = math.pi

NB_ROBOTS = 9
FOV = 0.2
ANGLE = 2.356
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

coord = (0, 0)
prev = ()
#m = rows
m = NB_ROBOTS
#n = col
n = 2
inBoid = [0 for i in range(NB_ROBOTS)];

robot_node = [[0]*n for i in range(NB_ROBOTS)]
orientationMatrix = [[0]*NB_ROBOTS for i in range(NB_ROBOTS)]
distMatrix = [[0]*NB_ROBOTS for i in range(NB_ROBOTS)]
trans_field = []
rota_field = []
emitter = robot.getEmitter("emitter")

for i in range(NB_ROBOTS):
    node = robot.getFromDef("EP"+str(i))
    tf = node.getField("translation")
    rf = node.getField("rotation")
    trans_field.append(tf)
    rota_field.append(rf)

def getCoordinates(trans_field):
    values = trans_field.getSFVec3f()
    coord = (values[0], values[2])
    return coord

def getRotation(rota_field):
    values = rota_field.getSFRotation()
    rota = values[3]
    return rota

# Don't use transform() before using this function
def avgRotation(*rotaList):
    rotation = 0
    total = 0
    for i in rotaList:
        for r in i:
            if (r != 999):
                rotation += r
                total += 1
    #return round(rotation / total, 5)
    if total == 0 :
        total = 1
    #return (rotation / total)
    #print(rotation / total)
    #return f'{(rotation / total):.5f}'
    #print(rotation / total)
    return rotation / total

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
    center = (round(x/total, 5), round(y/total, 5))
    center = (f'{(x/total):.5f}', f'{(y/total):.5f}')
    return center

def getDist(x1, y1, x2, y2):
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return dist

'''
def isBehind(x1, y1, x2, y2):
    x = x2-x1
    y = y2-y1
    Ax = x1 + FOV*math.cos(ANGLE)
    Ay = y1 + FOV*math.sin(ANGLE)
    Bx = x1 + FOV*math.cos(-ANGLE)
    By = y1 + FOV*math.sin(-ANGLE)
    angle = math.atan(x/y)

    print("A = {},{}; B = {},{}; t = {},{}".format(Ax,Ay, Bx, By, x2-x1, y2-y1))
'''

#transform [0;pi][0;-pi] range to [0,2pi]
def transform(rotation):
    if rotation < 0:
        rotation = 2*math.pi + rotation
        #print(rotation)
    return rotation


def rotationDiff(current, target):
    if target == 0:
        return f'{target:.5f}'
    delta = target - current
    #print("before = " + str(delta))
    if abs(delta) > PI:
        if current > target:
            delta = (2*PI + target) - current
        else:
            delta = target - (2*PI + current)
    return f'{delta:.5f}'

counter = 1

while (robot.step(timestep) != -1):
    for i in range(NB_ROBOTS):
        robot_node[i][0] = getCoordinates(trans_field[i])
        robot_node[i][1] = getRotation(rota_field[i])

    for i in range(NB_ROBOTS):
        for j in range(NB_ROBOTS):
            # register rotation angle of robots in range
            if ((getDist(*robot_node[i][0], *robot_node[j][0]) <= FOV) and i != j):
                inBoid[i] = 1
                orientationMatrix[i][j] = robot_node[j][1]
                distMatrix[i][j] = robot_node[j][0]
            else:
                orientationMatrix[i][j] = 999
                distMatrix[i][j] = 999

#try to only send relevant info only when necessary otherwise backlog in the robot
    if counter % 8 == 0:
        for i in range(NB_ROBOTS):
            r = "t" + rotationDiff(transform(robot_node[i][1]), transform(avgRotation(orientationMatrix[i])))
            b = "b" + str(inBoid[i])
            packet = [r, b]
            emitter.setChannel(i + 1)
            for item in packet:
                msg = bytes(item, 'utf-8')
                emitter.send(msg)
            counter += 1
            inBoid[i] = 0;
    else:
        counter += 1
    if counter > 1000:
        counter = 0
