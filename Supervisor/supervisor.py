"""Boids Supervisor."""

from controller import *
import math
import struct

NB_ROBOTS = 4
FOV = 0.2
robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

coord = (0, 0)
prev = ()
#m = rows
m = NB_ROBOTS
#n = col
n = 2

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

def rotationDiff(current, target):
    return f'{(target - current):.5f}'


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
    return f'{(rotation / total):.5f}'

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

counter = 1

while (robot.step(timestep) != -1):
    for i in range(NB_ROBOTS):
        robot_node[i][0] = getCoordinates(trans_field[i])
        robot_node[i][1] = getRotation(rota_field[i])

    for i in range(NB_ROBOTS):
        for j in range(NB_ROBOTS):
            # register rotation angle of robots in range
            if ((getDist(*robot_node[i][0], *robot_node[j][0]) <= FOV) and i != j):
                orientationMatrix[i][j] = robot_node[j][1]
                distMatrix[i][j] = robot_node[j][0]
            else:
                orientationMatrix[i][j] = 999
                distMatrix[i][j] = 999

#try to only send relevant info only when necessary otherwise backlog in the robot
    if counter % 8 == 0:
        for i in range(NB_ROBOTS):
            r = "t" + avgRotation(orientationMatrix[i])
            c = "c" + f'{(robot_node[i][1]):.5f}'
            #center = centerMass(distMatrix[i])
            #x ="x" + str(center[0])
            #y ="y" + str(center[1])
            #print(y)
            packet = [r, c]
            emitter.setChannel(i + 1)
            for item in packet:
                msg = bytes(item, 'utf-8')
                emitter.send(msg)
            counter += 1
    else:
        counter += 1
    if counter > 1000:
        counter = 0
