import sys
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt
import math
from numpy import dot

cMap2d = np.loadtxt(open("../test/unit/data/results/cMap3D.txt",'r'), skiprows=0)
cMap2d[np.where(cMap2d == 0)] = 0.0001       
cMap2d[np.isinf(cMap2d)] = 0       

path = np.loadtxt(open("../test/unit/data/results/roverPath.txt",'r'), skiprows=0)
path3D = np.loadtxt(open("../test/unit/data/results/EEPath.txt",'r'), skiprows=0)

xsize = 100
ysize = 100
zsize = 39
res = 0.1
resz = 0.1

costMap3D = np.ones([xsize, ysize, zsize])
c = 0
k = 0

for i in range(0, xsize):
    c = 0
    k = 0
    for j in range(0, ysize*zsize):
        costMap3D[k][i][c] = cMap2d[i][j]
        c += 1
        if c > zsize-1:
            c = 0
            k += 1

box = np.zeros([xsize, ysize, zsize])
box[0,:,:] = np.max(costMap3D)
box[-1,:,:] = np.max(costMap3D)
box[:,0,:] = np.max(costMap3D)
box[:,-1,:] = np.max(costMap3D)
box[:,:,0] = np.max(costMap3D)
box[:,:,-1] = np.max(costMap3D)

fig1 = mlab.figure()
mlab.plot3d(path[:,0]/res, path[:,1]/res, (1+0.625)/resz*np.ones(len(path)), color=(1,1,1), tube_radius = 1)
mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
mlab.contour3d(costMap3D, contours = 10, opacity = 0.7, transparent = True)
mlab.volume_slice(costMap3D, plane_orientation='y_axes', plane_opacity = 0.1, transparent = True)
mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)

fig2 = mlab.figure()
mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
mlab.volume_slice(costMap3D, plane_orientation='y_axes', plane_opacity = 0.1, transparent = True)
mlab.plot3d(path[:,0]/res, path[:,1]/res, (1+0.625)/resz*np.ones(len(path)), color=(1,1,1), tube_radius = 1)
mlab.plot3d(path3D[:,0]/res, path3D[:,1]/res, path3D[:,2]/resz, color=(0.3,0.3,0.5), tube_radius = 1)
mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)

mlab.show()
