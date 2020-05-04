"""Boids Supervisor."""

from controller import *
import math
import struct

NB_ROBOTS = 4
FOV = 0.15

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
#s = "test"
#msg = bytes("test", 'utf-8')
#e.send(msg)

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

def avgRotation(*rotaList):
    rotation = 0
    total = 0
    for i in rotaList:
        for r in i:
            if (r != 999):
                rotation += r
                total += 1
    #return round(rotation / total, 5)
    return f'{(rotation / total):.10f}'

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
    center = (round(x/total, 5), round(y/total, 5))
    center = (f'{(x/total):.10f}', f'{(y/total):.10f}')
    return center

def getDist(x1, y1, x2, y2):
    dist = math.sqrt((x2-x1)**2 + (y2-y1)**2)
    return dist


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
        #print("r -> i = {} -> {}".format(i, avgRotation(orientationMatrix[i])))
        #print("d -> i = {} -> {}".format(i, centerMass(distMatrix[i])))
        r = "r" + str(avgRotation(orientationMatrix[i]))
        c = centerMass(distMatrix[i])
        x ="x" + str(c[0])
        #print(x)
        y ="y" + str(c[1])
        #print(y)
        packet = [r, x, y]
        emitter.setChannel(i + 1)
        msg_r = bytes(r, 'utf-8')
        msg_x = bytes(x, 'utf-8')
        msg_y = bytes(y, 'utf-8')
        #emitter.send(msg_r)
        for item in packet:
            msg = bytes(item, 'utf-8')
            emitter.send(msg)



#    for i in range(NB_ROBOTS):
#        for j in range(NB_ROBOTS):
#            print("[{}][{}]".format(i, avgRotation(distMatrix[i])))


#    robo1 = robot_node[0][0]
#    robo2 = robot_node[1][0]
#    if (getDist(*robo1, *robo2) <= FOV):
#        rota_field[1].setSFRotation([0, 1, 0, robot_node[0][1]])


#while (robot.step(timestep) != -1):
#    del prev
#    (x, y) = coord
#    prev = (x, y)
#    del coord
#    values = trans_field.getSFVec3f()
#    rota = rota_field.getSFRotation()
#    coord = (values[0], values[2])
#    print("E-puck rotation : {} {} {} {}".format(rota[0], rota[1], rota[2], rota[3]))
#    print("x = {}, y = {}, prev_x = {}, prev_y = {}".format(coord[0], coord[1], prev[0], prev[1]))
