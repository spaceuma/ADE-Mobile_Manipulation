
import sys
import numpy as np
import matplotlib.pyplot as plt
#import scipy.ndimage
#from mayavi import mlab
import plotly.graph_objects as go

sizes = np.loadtxt(open("../data/planner/reachabilityDistances.txt",'r'), max_rows=1)
#sizes = np.loadtxt(open("../data/planner/reachabilityMap.txt",'r'), max_rows=1)
#sizes = np.loadtxt(open("../data/planner/reachabilityMap_Coupled.txt",'r'), max_rows=1)
resolutions = np.loadtxt(open("../data/planner/reachabilityDistances.txt",'r'), skiprows = 1, max_rows=1)
#resolutions = np.loadtxt(open("../data/planner/reachabilityMap.txt",'r'), skiprows = 1, max_rows=1)
#resolutions = np.loadtxt(open("../data/planner/reachabilityMap_Coupled.txt",'r'), skiprows = 1, max_rows=1)
minValues = np.loadtxt(open("../data/planner/reachabilityDistances.txt",'r'), skiprows = 2, max_rows=1)
#minValues = np.loadtxt(open("../data/planner/reachabilityMap.txt",'r'), skiprows = 2, max_rows=1)
#minValues = np.loadtxt(open("../data/planner/reachabilityMap_Coupled.txt",'r'), skiprows = 2, max_rows=1)
reachabilityMap2D = np.loadtxt(open("../data/planner/reachabilityDistances.txt",'r'), skiprows=3)
#reachabilityMap2D = np.loadtxt(open("../data/planner/reachabilityMap.txt",'r'), skiprows=3)
#reachabilityMap2D = np.loadtxt(open("../data/planner/reachabilityMap_Coupled.txt",'r'), skiprows=3)

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

#fig1 = mlab.figure()
#mlab.contour3d(reachabilityMap3D, contours = 2, opacity = 1, transparent = False)
#mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([20]), np.array([0]), np.array([0]), scale_factor = 0.3, color = (1,0,0))
#mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([0]), np.array([20]), np.array([0]), scale_factor = 0.3, color = (0,1,0))
#mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([0]), np.array([0]), np.array([20]), scale_factor = 0.3, color = (0,0,1))
#mlab.volume_slice(reachabilityMap3D, plane_orientation='x_axes', opacity = 0, plane_opacity = 0, transparent = True)
#mlab.show()

stopx = xsize*resXY+minValues[0]
stopy = ysize*resXY+minValues[1]
stopz = zsize*resZ+minValues[2]
complexSizex = complex(0,xsize)
complexSizey = complex(0,ysize)
complexSizez = complex(0,zsize)


X, Y, Z = np.mgrid[minValues[0]:stopx:complexSizex,minValues[1]:stopy:complexSizey,minValues[2]:stopz:complexSizez]

fig = go.Figure(data=go.Volume(
    x = X.flatten(),
    y=Y.flatten(),
    z=Z.flatten(),
    value=reachabilityMap3D.flatten(),
    isomin=0.001,
    isomax=1,
    opacity=0.1, # needs to be small to see through all surfaces
    surface_count=2, # needs to be a large number for good volume rendering
    ))
fig.show()
