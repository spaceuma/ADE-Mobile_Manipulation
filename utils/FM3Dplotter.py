
import sys
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt

costMap2d = np.loadtxt(open("../test/unit/data/dummyCostMap3D.txt",'r'), skiprows=0)

TMapStart2d = np.loadtxt(open("../test/unit/data/results/TMapStart3D.txt",'r'), skiprows=0)
TMapStart2d[np.where(TMapStart2d == 0)] = 0.0001       
TMapStart2d[np.isinf(TMapStart2d)] = 0       

TMapGoal2d = np.loadtxt(open("../test/unit/data/results/TMapGoal3D.txt",'r'), skiprows=0)
TMapGoal2d[np.where(TMapGoal2d == 0)] = 0.0001       
TMapGoal2d[np.isinf(TMapGoal2d)] = 0       

path = np.loadtxt(open("../test/unit/data/results/3Dpath.txt"), skiprows=0)

xsize = 100
ysize = 100
zsize = 100

costMap = np.ones([xsize, ysize, zsize])
TMapStart = np.ones([xsize, ysize, zsize])
TMapGoal = np.ones([xsize, ysize, zsize])
c = 0
k = 0

for i in range(0, xsize):
    c = 0
    k = 0
    for j in range(0, ysize*zsize):
        costMap[i][k][c] = costMap2d[i][j]
        TMapStart[i][k][c] = TMapStart2d[i][j]
        TMapGoal[i][k][c] = TMapGoal2d[i][j]
        c += 1
        if c > zsize-1:
            c = 0
            k += 1

box = np.zeros([xsize, ysize, zsize])
box[0,6:54,6:94] = np.max(costMap)
box[24,6:54,6:94] = np.max(costMap)
box[0:24,6,6:94] = np.max(costMap)
box[0:24,54,6:94] = np.max(costMap)
box[0:24,6:54,6] = np.max(costMap)
box[0:24,6:54,94] = np.max(costMap)

TMap = TMapStart+TMapGoal
TMap[np.where(TMap == 0)] = np.max(TMap)
TMap = 1-np.dot(TMap, 1/np.max(TMap))

fig1 = mlab.figure()
mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
mlab.contour3d(costMap, contours = 10, opacity = 0.7, transparent = True)
mlab.plot3d(path[:,1], path[:,0], path[:,2], color=(1,1,1), tube_radius = 1)
#mlab.volume_slice(TMap, plane_orientation='x_axes', plane_opacity = 0.1, transparent = True)
mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)

fig2 = mlab.figure()
mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
mlab.contour3d(TMap, contours = 30, opacity = 0.15, transparent = True)
mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)

mlab.show()
