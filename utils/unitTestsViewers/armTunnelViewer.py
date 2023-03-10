import sys
import numpy as np
from mayavi import mlab
import matplotlib.pyplot as plt
import math
from numpy import dot
import moviepy.editor as mpy

if len(sys.argv) != 2:
  print('ERROR. The script needs one additional input:')
  print('0 for END deployment')
  print('1 for PROGRESSIVE deployment')
  print('2 for BEGINNING deployment')
  print('3 for DECOUPLED solution')
  sys.exit()

representationNumber = str(int(sys.argv[1])+1)

if (int(representationNumber) < 1) or (int(representationNumber) > 4):
  print('ERROR. The provided input is not valid:')
  print('0 for END deployment')
  print('1 for PROGRESSIVE deployment')
  print('2 for BEGINNING deployment')
  print('3 for DECOUPLED solution')
  sys.exit()

if int(representationNumber) == 1:
  print('Case END deployment')
elif int(representationNumber) == 2:
  print('Case PROGRESSIVE deployment')
elif int(representationNumber) == 3:
  print('Case BEGINNING deployment')
elif int(representationNumber) == 4:
  print('Case DECOUPLED deployment')

cMap2d = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_3dmap_0"+representationNumber+".txt",'r'), skiprows=1)
cMap2d[np.where(cMap2d == 0)] = 0.0001
cMap2d[np.isinf(cMap2d)] = 0

sizes = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_3dmap_0"+representationNumber+".txt",'r'), max_rows=1)
resolutions = np.loadtxt(open("../../test/unit/data/input/MMMotionPlanTest/res_info.txt",'r'), max_rows=1)

path = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_path_0"+representationNumber+".txt",'r'), skiprows=0)
path3D = np.loadtxt(open("../../test/unit/data/results/MMMotionPlanTest/nominal_working_eepath_0"+representationNumber+".txt",'r'), skiprows=0)

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

"""box = np.zeros([ysize, xsize, zsize])
box[0,:,:] = np.max(costMap3D)
box[-1,:,:] = np.max(costMap3D)
box[:,0,:] = np.max(costMap3D)
box[:,-1,:] = np.max(costMap3D)
box[:,:,0] = np.max(costMap3D)
box[:,:,-1] = np.max(costMap3D)"""

d = 0
for i in range(1,len(path)):
    d = d + np.linalg.norm(path[i]-path[i-1])

print('Distance covered in the path: '+str(d))

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

DEM = np.loadtxt(open("../../test/unit/data/input/MMMotionPlanTest/GalopprennbahnWest_Zone01_10cmDEM.csv",'r'), skiprows=0)

minz = np.min(DEM[:,:])
DEM0 = DEM[:,:] - minz

xMap= np.linspace(0,res*xsize,xsize)
yMap= np.linspace(0,res*ysize,ysize)
x,y = np.meshgrid(xMap,yMap)


fig1 = mlab.figure(size=(500,500), bgcolor=(1,1,1))
surf = mlab.surf(xMap,yMap, np.flipud(np.rot90(DEM0)), colormap = 'gist_earth') #np.flipud(np.fliplr(DEM0)))
lut = surf.module_manager.scalar_lut_manager.lut.table.to_array()
lut[:,0] = np.linspace(246, 100, 256)
lut[:,1] = np.linspace(215, 154, 256)
lut[:,2] = np.linspace(176, 23, 256)
surf.module_manager.scalar_lut_manager.lut.table = lut
mlab.plot3d(path[:,0], path[:,1], path[:,2], color=(0,0,1), tube_radius = 0.05)
mlab.contour3d(X,Y,Z,costMap3D/np.max(costMap3D), contours = 10, opacity = 0.04, transparent = True, color = (105/255,176/255,250/255), vmax = 0.99, vmin = 0.001)
#mlab.volume_slice(costMap3D, plane_orientation='y_axes', plane_opacity = 0.1, transparent = True)
mlab.quiver3d(0, 0, 0, 1, 0, 0, scale_factor = 1, color=(1,0,0))
mlab.quiver3d(0, 0, 0, 0, 1, 0, scale_factor = 1, color=(0,1,0))
mlab.quiver3d(0, 0, 0, 0, 0, 1, scale_factor = 1, color=(0,0,1))
mlab.points3d(path3D[-1][0], path3D[-1][1], DEM0[int(path3D[-1][1]/res+0.5),int(path3D[-1][0]/res+0.5)], scale_factor = 0.2, color=(50/255,50/255,50/255), mode = 'cube')
#mlab.quiver3d(path3D[-1][0], path3D[-1][1], path3D[-1][2],1,0,0, scale_factor = 0.2, color=(1,0,0))
#mlab.quiver3d(path3D[-1][0], path3D[-1][1], path3D[-1][2],0,1,0, scale_factor = 0.2, color=(0,1,0))
#mlab.quiver3d(path3D[-1][0], path3D[-1][1], path3D[-1][2],0,0,1, scale_factor = 0.2, color=(0,0,1))

mlab.show()
