import sys
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt

cMap2d = np.loadtxt(open("../test/unit/data/results/cMap3D.txt",'r'), skiprows=0)
cMap2d[np.where(cMap2d == 0)] = 0.0001       
cMap2d[np.isinf(cMap2d)] = 0       

path = np.loadtxt(open("../test/unit/data/results/roverPath.txt"), skiprows=0)
path3D = np.loadtxt(open("../test/unit/data/results/EEPath.txt"), skiprows=0)


xsize = 100
ysize = 100
zsize = 39

costMap3D = np.ones([xsize, ysize, zsize])
c = 0
k = 0

for i in range(0, xsize):
    c = 0
    k = 0
    for j in range(0, ysize*zsize):
        costMap3D[i][k][c] = cMap2d[i][j]
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
mlab.plot3d(path[:,1]/0.1, path[:,0]/0.1, (10+6.25)*np.ones(len(path)), color=(1,1,1), tube_radius = 1)
mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
mlab.contour3d(costMap3D, contours = 10, opacity = 0.7, transparent = True)
mlab.volume_slice(costMap3D, plane_orientation='x_axes', plane_opacity = 0.1, transparent = True)
mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)

fig2 = mlab.figure()
mlab.plot3d(path[:,1]/0.1, path[:,0]/0.1, (10+6.25)*np.ones(len(path)), color=(1,1,1), tube_radius = 1)
mlab.plot3d(path3D[:,1]/0.1, path3D[:,0]/0.1, path3D[:,2]/0.1, color=(0,0,1), tube_radius = 0.8)
mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
mlab.volume_slice(costMap3D, plane_orientation='x_axes', plane_opacity = 0.1, transparent = True)
mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)

mlab.show()
