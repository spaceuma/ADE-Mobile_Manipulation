import sys
import numpy as np
from mayavi import mlab
print("Imported mayavi")
import matplotlib.pyplot as plt
print("Imported plt")
import math
print("Imported math")
from numpy import dot
#import moviepy.editor as mpy
#print("Imported moviepy")

cMap2d = np.loadtxt(open("./extractedLog/3dcostMap.txt",'r'), skiprows=1)
cMap2d[np.where(cMap2d == 0)] = 0.0001
cMap2d[np.isinf(cMap2d)] = 0

sizes = np.loadtxt(open("./extractedLog/3dcostMap.txt",'r'), max_rows=1)

path = np.loadtxt(open("./extractedLog/roverPath.txt",'r'), skiprows=0)
path3D = np.loadtxt(open("./extractedLog/wristPath.txt",'r'), skiprows=0)

xsize = int(sizes[1])
ysize = int(sizes[0])
zsize = int(sizes[2])

print("WARNING: using default resolutions: 0.1, 0.1, 0.08 m")
res = 0.1
resz = 0.08

costMap3D = np.zeros([xsize, ysize, zsize])

c = 0
k = 0

for i in range(2, ysize):
    c = 0
    k = 0
    for j in range(0, xsize*zsize):
        #print(i)
        #print(j)
        #print("-")
        costMap3D[k][i][c] = cMap2d[i][j]
        c += 1
        if c > zsize-1:
            c = 0
            k += 1

'''box = np.zeros([xsize, ysize, zsize])
box[0,:,:] = np.max(costMap3D)
box[-1,:,:] = np.max(costMap3D)
box[:,0,:] = np.max(costMap3D)
box[:,-1,:] = np.max(costMap3D)
box[:,:,0] = np.max(costMap3D)
box[:,:,-1] = np.max(costMap3D)'''

d = 0
for i in range(1,len(path)):
    d = d + np.linalg.norm(path[i]-path[i-1])

stopx = xsize*res
stopy = ysize*res
stopz = zsize*resz

complexSizex = complex(0,xsize)
complexSizey = complex(0,ysize)
complexSizez = complex(0,zsize)
"""x = np.linspace(0,stopx,xsize)
y = np.linspace(0,stopy,ysize)
z = np.linspace(0,stopz,zsize)"""

X, Y, Z = np.mgrid[0:stopy:complexSizey,0:stopx:complexSizex,0:stopz:complexSizez]

DEM = np.loadtxt(open("./extractedLog/elevationMap.txt",'r'), skiprows=0)

minz = np.min(DEM[:,:])
DEM0 = DEM[:,:] - minz

xMap= np.linspace(0,res*xsize,xsize)
yMap= np.linspace(0,res*ysize,ysize)
x,y = np.meshgrid(xMap,yMap)


fig1 = mlab.figure(size=(500,500), bgcolor=(1,1,1))
mlab.surf(xMap,yMap, np.flipud(np.rot90(DEM0)), color = (193/255,68/255,14/255)) #np.flipud(np.fliplr(DEM0)))
mlab.plot3d(path[:,0], path[:,1], path[:,2], color=(0,0,1), tube_radius = 0.05)
mlab.contour3d(X,Y,Z,costMap3D/np.max(costMap3D), contours = 10, opacity = 0.04, transparent = True, color = (105/255,176/255,250/255), vmax = 0.99, vmin = 0.001)
#mlab.volume_slice(costMap3D, plane_orientation='y_axes', plane_opacity = 0.1, transparent = True)
mlab.quiver3d(0, 0, 0, 1, 0, 0, scale_factor = 1, color=(1,0,0))
mlab.quiver3d(0, 0, 0, 0, 1, 0, scale_factor = 1, color=(0,1,0))
mlab.quiver3d(0, 0, 0, 0, 0, 1, scale_factor = 1, color=(0,0,1))
mlab.points3d(path3D[-1][0], path3D[-1][1], DEM0[int(path3D[-1][0]/res+0.5),int(path3D[-1][1]/res+0.5)], scale_factor = 0.2, color=(192/255,192/255,192/255))
#mlab.quiver3d(path3D[-1][0], path3D[-1][1], path3D[-1][2],1,0,0, scale_factor = 0.2, color=(1,0,0))
#mlab.quiver3d(path3D[-1][0], path3D[-1][1], path3D[-1][2],0,1,0, scale_factor = 0.2, color=(0,1,0))
#mlab.quiver3d(path3D[-1][0], path3D[-1][1], path3D[-1][2],0,0,1, scale_factor = 0.2, color=(0,0,1))

mlab.show()
