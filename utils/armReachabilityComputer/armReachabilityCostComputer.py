# WARNING #
# Installing scipy may break mayavi animations like the ones in armAnimator.py,
# uninstall scipy and the animations should work again.
# pip3 install scipy
# pip3 uninstall scipy

import sys
import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage
from mayavi import mlab

sizes = np.loadtxt(open("../../data/planner/reachabilityMap_Coupled.txt",'r'), max_rows=1)
resolutions = np.loadtxt(open("../../data/planner/reachabilityMap_Coupled.txt",'r'), skiprows = 1, max_rows=1)
minValues = np.loadtxt(open("../../data/planner/reachabilityMap_Coupled.txt",'r'), skiprows = 2, max_rows=1)
reachabilityMap2D = np.loadtxt(open("../../data/planner/reachabilityMap_Coupled.txt",'r'), skiprows=3)

xsize = int(sizes[0])
ysize = int(sizes[1])
zsize = int(sizes[2])

resXY = resolutions[0]
resZ = resolutions[2]

reachabilityMap3D = np.zeros([xsize, ysize, zsize])
c = 0
k = 0

for i in range(2, xsize):
    c = 0
    k = 0
    for j in range(0, ysize*zsize):
        reachabilityMap3D[i][k][c] = reachabilityMap2D[i][j]
        c += 1
        if c > zsize-1:
            c = 0
            k += 1

costs = np.zeros([np.size(reachabilityMap3D,0), np.size(reachabilityMap3D,1), np.size(reachabilityMap3D,2)])
reachabilityMapSafe = np.zeros([np.size(reachabilityMap3D,0), np.size(reachabilityMap3D,1), np.size(reachabilityMap3D,2)])
for i in range(0,np.size(reachabilityMap3D,0)):
  for j in range(0,np.size(reachabilityMap3D,1)):
    for k in range(0,np.size(reachabilityMap3D,2)):
      if reachabilityMap3D[i][j][k] == 2:
        reachabilityMapSafe[i][j][k] = 1
slice_ = np.zeros([np.size(reachabilityMap3D,1), np.size(reachabilityMap3D,2)])

for i in range(0,np.size(reachabilityMap3D,0)):
  slice_ = reachabilityMapSafe[i][:][:]
  costs[i][:][:] = scipy.ndimage.morphology.distance_transform_edt(np.array(slice_), sampling = [resXY, resZ])

# Write the array to disk
with open('../../data/planner/reachabilityDistances_Coupled.txt', 'w') as outfile:
    np.savetxt(outfile, sizes, fmt='%-1d', newline=" ")
    outfile.write("\n")
    np.savetxt(outfile, resolutions, fmt='%-4f', newline=" ")
    outfile.write("\n")
    np.savetxt(outfile, minValues, fmt='%-4f', newline=" ")
    outfile.write("\n")
    for i in range(0, xsize):
        c = 0
        k = 0
        for j in range(0, ysize):
            np.savetxt(outfile, costs[i][j][:], fmt='%-4f', newline=" ")
        outfile.write("\n")

### Deprecated representation, better use armReachabilityMapViewer.py
'''fig = mlab.figure()
mlab.contour3d(costs, contours = 20, opacity = 1, transparent = True)
mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([20]), np.array([0]), np.array([0]), scale_factor = 0.3, color = (1,0,0))
mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([0]), np.array([20]), np.array([0]), scale_factor = 0.3, color = (0,1,0))
mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([0]), np.array([0]), np.array([20]), scale_factor = 0.3, color = (0,0,1))
mlab.volume_slice(costs, plane_orientation='x_axes', opacity = 0, plane_opacity = 0, transparent = True)
mlab.show()'''
