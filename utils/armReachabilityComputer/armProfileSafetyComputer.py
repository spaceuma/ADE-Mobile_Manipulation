# WARNING #
# Installing scipy may break mayavi animations like the ones in armAnimator.py,
# uninstall scipy and the animations should work again.
# pip3 install scipy
# pip3 uninstall scipy

import sys
import numpy as np
import matplotlib.pyplot as plt
import math
from numpy import dot
import scipy.ndimage
from mayavi import mlab

def traslation(p):

    # Traslation transformation matrix

    D = [[1, 0, 0, p[0]],
         [0, 1, 0, p[1]],
         [0, 0, 1, p[2]],
         [0, 0, 0, 1]]

    return D

def rotZ(theta):

    # Z rotation transformation matrix

    s = math.sin(theta)
    c = math.cos(theta)

    if abs(c) < 0.000000001: c = 0
    if abs(s) < 0.000000001: s = 0

    RZ = [[c, -s, 0, 0],
          [s, c,  0, 0],
          [0, 0,  1, 0],
          [0, 0,  0, 1]]

    return RZ

def rotY(theta):

    # Y rotation transformation matrix

    s = math.sin(theta)
    c = math.cos(theta)

    if abs(c) < 0.000000001: c = 0
    if abs(s) < 0.000000001: s = 0

    RY = [[c,  0, s, 0],
          [0,  1, 0, 0],
          [-s, 0, c, 0],
          [0,  0, 0, 1]]

    return RY

def rotX(theta):

    # X rotation transformation matrix

    s = math.sin(theta)
    c = math.cos(theta)

    if abs(c) < 0.000000001: c = 0
    if abs(s) < 0.000000001: s = 0

    RX = [[1, 0, 0,  0],
          [0, c, -s, 0],
          [0, s, c,  0],
          [0, 0, 0,  1]]

    return RX

def plotArm(q,pos,heading):

    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2] + math.pi/2
    theta4 = q[3]
    theta5 = q[4]
    theta6 = q[5]

    # Manipulator parameters
    d0 = 0.500
    a1 = 0.225
    a2 = 0.735
    c2 = 0.030
    a3 = 0.030
    d4 = 0.695
    d6 = 0.300

    # Generating transformation matrices for each joint
    TwB = dot(traslation(pos),dot(rotZ(heading[2]) ,dot(rotY(heading[1]),rotX(heading[0]))))
    TB0 = traslation([0,0,d0-c2])
    T01 = dot(traslation([0,0,c2]), dot(rotZ(theta1), dot(traslation([a1,0,0]), rotX(-math.pi/2))))
    T1a = dot(traslation([0,0,0]), dot(rotZ(theta2), dot(traslation([a2-c2,0,0]), rotX(0))))
    T12 = dot(traslation([0,0,0]), dot(rotZ(theta2), dot(traslation([a2,c2,0]), rotX(0))))
    T23 = dot(traslation([0,0,0]), dot(rotZ(theta3), dot(traslation([-a3,0,0]), rotX(math.pi/2))))
    T23v = dot(traslation([0,0,0]), dot(rotZ(theta3), dot(traslation([-a3,-c2,0]), rotX(math.pi/2))))
    T3b = traslation([0,0,d4/2])
    T34 = dot(traslation([0,0,d4]), dot(rotZ(theta4), dot(traslation([0,0,0]), rotX(-math.pi/2))))
    T45 = dot(traslation([0,0,0]), dot(rotZ(theta5), dot(traslation([0,0,0]), rotX(math.pi/2))))
    T56 = dot(traslation([0,0,d6]), dot(rotZ(theta6), dot(traslation([0,0,0]), rotX(0))))


    # The transformation matrix from base to end-effector
    Tw0 = dot(TwB,TB0)
    Tw1 = dot(Tw0,T01)
    Tw2 = dot(Tw1,T12)
    Twa = dot(Tw1,T1a)
    Tw3 = dot(Tw2,T23)
    Tw3v = dot(Tw2,T23v)
    Twb = dot(Tw3,T3b)
    Tw4 = dot(Tw3,T34)
    Tw5 = dot(Tw4,T45)
    Tw6 = dot(Tw5,T56)

    px=[TwB[0][3],Tw0[0][3],Tw1[0][3],Twa[0][3],Tw2[0][3],Tw3v[0][3],Twb[0][3],Tw4[0][3],Tw5[0][3],Tw6[0][3]]
    py=[TwB[1][3],Tw0[1][3],Tw1[1][3],Twa[1][3],Tw2[1][3],Tw3v[1][3],Twb[1][3],Tw4[1][3],Tw5[1][3],Tw6[1][3]]
    pz=[TwB[2][3],Tw0[2][3],Tw1[2][3],Twa[2][3],Tw2[2][3],Tw3v[2][3],Twb[2][3],Tw4[2][3],Tw5[2][3],Tw6[2][3]]

    return px,py,pz

def DKMwrist(q,pos,heading):

    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2] + math.pi/2
    theta4 = q[3]
    theta5 = q[4]
    theta6 = q[5]

    # Manipulator parameters
    d0 = 0.500
    a1 = 0.225
    a2 = 0.735
    c2 = 0.030
    a3 = 0.030
    d4 = 0.695
    d6 = 0.300

    # Generating transformation matrices for each joint
    TwB = dot(traslation(pos),dot(rotZ(heading[2]) ,dot(rotY(heading[1]),rotX(heading[0]))))
    TB0 = traslation([0,0,d0])
    T01 = dot(traslation([0,0,0]), dot(rotZ(theta1), dot(traslation([a1,0,0]), rotX(-math.pi/2))))
    T12 = dot(traslation([0,0,0]), dot(rotZ(theta2), dot(traslation([a2,c2,0]), rotX(0))))
    T23 = dot(traslation([0,0,0]), dot(rotZ(theta3), dot(traslation([-a3,0,0]), rotX(math.pi/2))))
    T34 = dot(traslation([0,0,d4]), rotX(-math.pi/2))


    # The transformation matrix from base to end-effector
    Tw0 = dot(TwB,TB0)
    Tw1 = dot(Tw0,T01)
    Tw2 = dot(Tw1,T12)
    Tw3 = dot(Tw2,T23)
    Tw4 = dot(Tw3,T34)

    m, n = Tw4.shape
    for j in range(0, m):
        for i in range(0, n):
            if abs(Tw4[j, i]) < 1e-4:
                    Tw4[j, i] = 0

    return Tw4

def DKM(q,pos,heading):

    theta1 = q[0]
    theta2 = q[1]
    theta3 = q[2] + math.pi/2
    theta4 = q[3]
    theta5 = q[4]
    theta6 = q[5]

    # Manipulator parameters
    d0 = 0.500
    a1 = 0.225
    a2 = 0.735
    c2 = 0.030
    a3 = 0.030
    d4 = 0.695
    d6 = 0.300

    # Generating transformation matrices for each joint
    TwB = dot(traslation(pos),dot(rotZ(heading[2]) ,dot(rotY(heading[1]),rotX(heading[0]))))
    TB0 = traslation([0,0,d0])
    T01 = dot(traslation([0,0,0]), dot(rotZ(theta1), dot(traslation([a1,0,0]), rotX(-math.pi/2))))
    T12 = dot(traslation([0,0,0]), dot(rotZ(theta2), dot(traslation([a2,c2,0]), rotX(0))))
    T23 = dot(traslation([0,0,0]), dot(rotZ(theta3), dot(traslation([-a3,0,0]), rotX(math.pi/2))))
    T34 = dot(traslation([0,0,d4]), dot(rotZ(theta4), dot(traslation([0,0,0]), rotX(-math.pi/2))))
    T45 = dot(traslation([0,0,0]), dot(rotZ(theta5), dot(traslation([0,0,0]), rotX(math.pi/2))))
    T56 = dot(traslation([0,0,d6]), dot(rotZ(theta6), dot(traslation([0,0,0]), rotX(0))))


    # The transformation matrix from base to end-effector
    Tw0 = dot(TwB,TB0)
    Tw1 = dot(Tw0,T01)
    Tw2 = dot(Tw1,T12)
    Tw3 = dot(Tw2,T23)
    Tw4 = dot(Tw3,T34)
    Tw5 = dot(Tw4,T45)
    Tw6 = dot(Tw5,T56)

    m, n = Tw6.shape
    for j in range(0, m):
        for i in range(0, n):
            if abs(Tw6[j, i]) < 1e-4:
                    Tw6[j, i] = 0

    return Tw6

def IKM(position, orientation, shoulder = 1, elbow = 1):
    # Inputs
    x = position[0]
    y = position[1]
    z = position[2]

    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]

    if abs(shoulder) != 1:
        return 0

    if abs(elbow) != 1:
        return 0

    # Manipulator parameters
    d0 = 0.500
    a1 = 0.225
    a2 = 0.735
    c2 = 0.030
    a3 = 0.030
    d4 = 0.695
    d6 = 0.300

    Tor = dot(rotZ(yaw),dot(rotY(pitch),rotX(roll)))

    xm = x - d6 * Tor[0,2]
    ym = y - d6 * Tor[1,2]
    zm = z - d6 * Tor[2,2]

    theta1 = math.atan2(ym,xm) - math.pi/2 + shoulder*math.pi/2

    r = math.sqrt(xm**2 + ym**2) - shoulder*a1

    alpha = math.atan2(zm - d0, r)
    d00 = math.sqrt((zm-d0)**2 + r**2)
    d01 = math.sqrt(c2**2 + a2**2)
    d02 = math.sqrt(a3**2 + d4**2)
    #if d00 > a2+d4:
    #    print('Wrist position is too far: '+str(d00)+'>'+str(a2+d4))
    #    return 0

    beta = math.acos((d01**2 + d00**2 - d02**2)/(2*d01*d00))
    gamma = math.acos((d01**2 + d02**2 - d00**2)/(2*d01*d02))

    theta2ini = math.atan2(c2,a2)
    theta3ini = math.atan2(a3,d4)

    theta2 = 3*math.pi/2 + shoulder*math.pi/2  - shoulder*alpha - elbow*beta - theta2ini
    theta3 = math.pi - elbow*gamma + (theta3ini + theta2ini)

    T01 = dot(traslation([0,0,0]), dot(rotZ(theta1), dot(traslation([a1,0,0]), rotX(-math.pi/2))))
    T12 = dot(traslation([0,0,0]), dot(rotZ(theta2), dot(traslation([a2,c2,0]), rotX(0))))
    T23 = dot(traslation([0,0,0]), dot(rotZ(theta3), dot(traslation([0,-a3,0]), dot(rotX(math.pi/2),rotY(math.pi/2))))) # The last Y rotation fixes the theta3 initial pos issue
    T03 = dot(T01,dot(T12,T23)) 

    R03 = T03[0:3,0:3]
    R06 = Tor[0:3,0:3]
    R36 = dot(np.linalg.inv(R03),R06)
        
    c5 = R36[2,2]
    s5 = math.sqrt(R36[0,2]**2 + R36[1,2]**2)

    if abs(s5) >= 1e-4:
        c4 = R36[0,2]/s5
        s4 = R36[1,2]/s5

        c6 = -R36[2,0]/s5
        s6 = R36[2,1]/s5

        print('Standard case, s4 = '+str(s4)+', c4 = '+str(c4))
        print('Standard case, s5 = '+str(s5)+', c5 = '+str(c5))
        print('Standard case, s6 = '+str(s6)+', c6 = '+str(c6))
        theta4 = math.atan2(s4,c4)
        theta5 = math.atan2(s5,c5)
        theta6 = math.atan2(s6,c6)
    else:
        if c5 > 0:
            s46 = R36[1,0]
            c46 = R36[1,1]
        
            theta4 = 0
            theta5 = math.atan2(s5,c5)
            theta6 = math.atan2(s46,c46)            

        else:       
            s64 = R36[1,0]
            c64 = R36[1,1]
        
            theta4 = 0
            theta5 = math.atan2(s5,c5)
            theta6 = math.atan2(s64,c64)            
             

    if theta1 >= math.pi:
        theta1 = theta1 - 2*math.pi
    if theta1 <= -math.pi:
        theta1 = theta1 + 2*math.pi

    if theta2 >= math.pi:
        theta2 = theta2 - 2*math.pi
    if theta2 <= -math.pi:
        theta2 = theta2 + 2*math.pi

    if theta3 >= math.pi:
        theta3 = theta3 - 2*math.pi
    if theta3 <= -math.pi:
        theta3 = theta3 + 2*math.pi

    if theta4 >= math.pi:
        theta4 = theta4 - 2*math.pi
    if theta4 <= -math.pi:
        theta4 = theta4 + 2*math.pi

    if theta5 >= math.pi:
        theta5 = theta5 - 2*math.pi
    if theta5 <= -math.pi:
        theta5 = theta5 + 2*math.pi

    if theta6 >= math.pi:
        theta6 = theta6 - 2*math.pi
    if theta6 <= -math.pi:
        theta6 = theta6 + 2*math.pi

    if abs(theta1) <= 1e-4:
        theta1 = 0
    if abs(theta2) <= 1e-4:
        theta2 = 0
    if abs(theta3) <= 1e-4:
        theta3 = 0
    if abs(theta4) <= 1e-4:
        theta4 = 0
    if abs(theta5) <= 1e-4:
        theta5 = 0
    if abs(theta6) <= 1e-4:
        theta6 = 0

    return theta1, theta2, theta3, theta4, theta5, theta6


sizes = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), max_rows=1)
resolutions = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), skiprows = 1, max_rows=1)
minValues = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), skiprows = 2, max_rows=1)
reachabilityDistance2D = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), skiprows=3)

armJoints = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_profile_03.txt",'r'), skiprows=0)

xsize = int(sizes[0])
ysize = int(sizes[1])
zsize = int(sizes[2])

resXY = resolutions[0]
resZ = resolutions[2]

reachabilityDistance3D = np.zeros([xsize, ysize, zsize])
c = 0
k = 0

for i in range(2, xsize):
    c = 0
    k = 0
    for j in range(0, ysize*zsize):
        reachabilityDistance3D[i][k][c] = reachabilityDistance2D[i][j]
        c += 1
        if c > zsize-1:
            c = 0
            k += 1

pathSize = np.size(armJoints, 0)
numJoints = np.size(armJoints, 1)
pos = [0,0,0]
heading = [0,0,0]
totalDistToCollisions = 0

for i in range(0, pathSize):
    TW4= DKMwrist(armJoints[i], pos, heading)
    indexRV = [int((TW4[0,3] - minValues[0])/resXY + 0.5), int((TW4[1,3] - minValues[1])/resXY + 0.5), int((TW4[2,3] - minValues[2])/resZ + 0.5)]
    distToCollision = reachabilityDistance3D[indexRV[0], indexRV[1], indexRV[2]]
    if distToCollision == 0:
        print("Collision detected!")
        print("In config: " + str(armJoints[i]))
        print("with wrist position: " + str(TW4[0,3]) + " " + str(TW4[1,3]) +" " + str(TW4[2,3]))
    totalDistToCollisions = totalDistToCollisions + distToCollision

avgDistToCollisions = totalDistToCollisions/pathSize
print("Average distance to self-collisions in profile: " + str(avgDistToCollisions))
