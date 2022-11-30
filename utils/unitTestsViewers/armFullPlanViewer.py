import sys
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt
import math
from numpy import dot
import moviepy.editor as mpy
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

if len(sys.argv) != 2:
  print('ERROR: the script needs one additional input:')
  print('0 for END deployment')
  print('1 for PROGRESSIVE deployment')
  print('2 for BEGINNING deployment')
  sys.exit()

representationNumber = str(int(sys.argv[1])+1)

cMap2d = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_3dmap_0"+representationNumber+".txt",'r'), skiprows=1)
cMap2d[np.where(cMap2d == 0)] = 0.0001
cMap2d[np.isinf(cMap2d)] = 0

sizes = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_3dmap_0"+representationNumber+".txt",'r'), max_rows=1)
resolutions = np.loadtxt(open("../../test/unit/data/input/MMMotionPlanTest/res_info.txt",'r'), max_rows=1)

path = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_path_0"+representationNumber+".txt",'r'), skiprows=0)
path3D = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_eepath_0"+representationNumber+".txt",'r'), skiprows=0)

xsize = int(sizes[0])
ysize = int(sizes[1])
zsize = int(sizes[2])

res = resolutions[0]
resz = resolutions[1]

costMap3D = np.zeros([ysize, xsize, zsize])
c = 0
k = 0

for i in range(2, xsize):
    c = 0
    k = 0
    for j in range(0, ysize*zsize):
        costMap3D[k][i][c] = cMap2d[i][j]
        c += 1
        if c > zsize-1:
            c = 0
            k += 1

armJoints = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_shadowing_profile_0"+representationNumber+".txt",'r'), skiprows=0)

DEM = np.loadtxt(open("../../test/unit/data/input/MMMotionPlanTest/RH1_Zone1_10cmDEM.csv",'r'), skiprows=0)

minz = np.min(DEM[:,:])
DEM0 = DEM[:,:] - minz

xMap= np.linspace(0,res*ysize,ysize)
yMap= np.linspace(0,res*xsize,xsize)
x,y = np.meshgrid(xMap,yMap)

d = np.zeros(len(path))
for i in range(1,len(path)):
    d[i] = d[i-1] + np.linalg.norm(path[i,0:2]-path[i-1,0:2])

stopx = xsize*res
stopy = ysize*res
stopz = zsize*resz

complexSizex = complex(0,xsize)
complexSizey = complex(0,ysize)
complexSizez = complex(0,zsize)
"""x = np.linspace(0,stopx,xsize)
y = np.linspace(0,stopy,ysize)
z = np.linspace(0,stopz,zsize)"""

X, Y, Z = np.mgrid[0:stopx:complexSizex,0:stopy:complexSizey,0:stopz:complexSizez]

fig1 = mlab.figure(size=(500,500), bgcolor=(1,1,1))
#mlab.mesh(x,y,DEM0, color = (231/255,125/255,17/255))
mlab.surf(xMap,yMap, np.flipud(np.rot90(DEM0)), color = (193/255,68/255,14/255)) #np.flipud(np.fliplr(DEM0)))
#mlab.view(azimuth = -110, elevation = 50, distance = 1000)
#mlab.view(-59, 58, 1773, [-.5, -.5, 512])
mlab.plot3d(path[:,0], path[:,1], path[:,2], color=(0,0,1), tube_radius = 0.04)
mlab.plot3d(path3D[:,0], path3D[:,1], path3D[:,2], color=(1,1,0), tube_radius = 0.04)
mlab.points3d(path3D[-1][0], path3D[-1][1], DEM0[int(path3D[-1][0]/res+0.5),int(path3D[-1][1]/res+0.5)], scale_factor = 0.2, color=(192/255,192/255,192/255))
mlab.quiver3d(0, 0, 0, 1, 0, 0, scale_factor = 1, color=(1,0,0))
mlab.quiver3d(0, 0, 0, 0, 1, 0, scale_factor = 1, color=(0,1,0))
mlab.quiver3d(0, 0, 0, 0, 0, 1, scale_factor = 1, color=(0,0,1))

n = np.size(armJoints,0)
for i in range(0,n,round(n/5)):
  T = DKM(armJoints[i,:], path[i,np.array([0,1,2])], [0,0,path[i,3]])
  rotT = T
  rotT[0,3] = 0   
  rotT[1,3] = 0   
  rotT[2,3] = 0   

  Tbx = dot(rotZ(path[i,3]), traslation([1,0,0]))
  Tby = dot(rotZ(path[i,3]), traslation([0,1,0]))
  Tbz = dot(rotZ(path[i,3]), traslation([0,0,1]))

  Tx = dot(rotT, traslation([1,0,0]))
  Ty = dot(rotT, traslation([0,1,0]))
  Tz = dot(rotT, traslation([0,0,1]))

  px,py,pz = plotArm(armJoints[i,:], path[i,np.array([0,1,2])], [0,0,path[i,3]])
  px = np.array(px)
  py = np.array(py)
  pz = np.array(pz)

  plt_arm = mlab.plot3d(px,py,pz,color=(0.1,0.1,0.1), tube_radius = 0.04)
  plt_joints = mlab.points3d(px[np.array([1,2,4,6,7,8])],py[np.array([1,2,4,6,7,8])],pz[np.array([1,2,4,6,7,8])],color=(0.8,0.8,0.8),scale_factor= 0.05)

  mlab.quiver3d(px[-1], py[-1], pz[-1], Tx[0,3], Tx[1,3], Tx[2,3], scale_factor = 0.3, color=(1,0,0))
  mlab.quiver3d(px[-1], py[-1], pz[-1], Ty[0,3], Ty[1,3], Ty[2,3], scale_factor = 0.3, color=(0,1,0))
  mlab.quiver3d(px[-1], py[-1], pz[-1], Tz[0,3], Tz[1,3], Tz[2,3], scale_factor = 0.3, color=(0,0,1))

  """mlab.quiver3d(px[0], py[0], pz[0], Tbx[0,3], Tbx[1,3], Tbx[2,3], scale_factor = 0.3, color=(1,0,0))
  mlab.quiver3d(px[0], py[0], pz[0], Tby[0,3], Tby[1,3], Tby[2,3], scale_factor = 0.3, color=(0,1,0))
  mlab.quiver3d(px[0], py[0], pz[0], Tbz[0,3], Tbz[1,3], Tbz[2,3], scale_factor = 0.3, color=(0,0,1))"""

T = DKM(armJoints[n-1,:], path[n-1,np.array([0,1,2])], [0,0,path[n-1,3]])
rotT = T
rotT[0,3] = 0   
rotT[1,3] = 0   
rotT[2,3] = 0   

Tbx = dot(rotZ(path[n-1,3]), traslation([1,0,0]))
Tby = dot(rotZ(path[n-1,3]), traslation([0,1,0]))
Tbz = dot(rotZ(path[n-1,3]), traslation([0,0,1]))

Tx = dot(rotT, traslation([1,0,0]))
Ty = dot(rotT, traslation([0,1,0]))
Tz = dot(rotT, traslation([0,0,1]))

px,py,pz = plotArm(armJoints[n-1,:], path[n-1,np.array([0,1,2])], [0,0,path[n-1,3]])
px = np.array(px)
py = np.array(py)
pz = np.array(pz)

plt_arm = mlab.plot3d(px,py,pz,color=(0.1,0.1,0.1), tube_radius = 0.04)
plt_joints = mlab.points3d(px[np.array([1,2,4,6,7,8])],py[np.array([1,2,4,6,7,8])],pz[np.array([1,2,4,6,7,8])],color=(0.8,0.8,0.8),scale_factor= 0.05)

mlab.quiver3d(px[-1], py[-1], pz[-1], Tx[0,3], Tx[1,3], Tx[2,3], scale_factor = 0.3, color=(1,0,0))
mlab.quiver3d(px[-1], py[-1], pz[-1], Ty[0,3], Ty[1,3], Ty[2,3], scale_factor = 0.3, color=(0,1,0))
mlab.quiver3d(px[-1], py[-1], pz[-1], Tz[0,3], Tz[1,3], Tz[2,3], scale_factor = 0.3, color=(0,0,1))

#mlab.volume_slice(X,Y,Z,costMap3D, plane_orientation='x_axes', opacity = 0, plane_opacity = 0, transparent = True)

"""mlab.quiver3d(px[0], py[0], pz[0], Tbx[0,3], Tbx[1,3], Tbx[2,3], scale_factor = 0.3, color=(1,0,0))
mlab.quiver3d(px[0], py[0], pz[0], Tby[0,3], Tby[1,3], Tby[2,3], scale_factor = 0.3, color=(0,1,0))
mlab.quiver3d(px[0], py[0], pz[0], Tbz[0,3], Tbz[1,3], Tbz[2,3], scale_factor = 0.3, color=(0,0,1))"""

mlab.show()

fig, ax = plt.subplots()
plt.scatter(d, armJoints[:, 0], label = 'First joint', s = 17)
plt.scatter(d, armJoints[:, 1], label = 'Second joint', s = 17)
plt.scatter(d, armJoints[:, 2], label = 'Third joint', s = 17)
plt.scatter(d, armJoints[:, 3], label = 'Fourth joint', s = 17)
plt.scatter(d, armJoints[:, 4], label = 'Fifth joint', s = 17)
plt.scatter(d, armJoints[:, 5], label = 'Sixth joint', s = 17)
#plt.title("Arm profile evolution along the trajectory", fontsize = 20)
plt.xlabel("Length of the trajectory covered (m)", fontsize = 20)
plt.ylabel("Joint position (rad)", fontsize = 20)
plt.legend(fontsize = 17)
plt.show()


