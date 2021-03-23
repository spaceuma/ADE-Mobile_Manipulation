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

box = np.zeros([xsize, ysize, zsize])
box[0,:,:] = np.max(costMap3D)
box[-1,:,:] = np.max(costMap3D)
box[:,0,:] = np.max(costMap3D)
box[:,-1,:] = np.max(costMap3D)
box[:,:,0] = np.max(costMap3D)
box[:,:,-1] = np.max(costMap3D)

d = 0
for i in range(1,len(path)):
    d = d + np.linalg.norm(path[i]-path[i-1])

print('Distance covered in the path: '+str(d))

fig1 = mlab.figure()
if path.ndim == 1:
    mlab.plot3d(path[0]/res, path[1]/res, path[2]/resz, color=(1,1,1), tube_radius = 1)
else:
    mlab.plot3d(path[:,0]/res, path[:,1]/res, path[:,2]/resz, color=(1,1,1), tube_radius = 1)

mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
mlab.contour3d(costMap3D, contours = 10, opacity = 0.7, transparent = True)
mlab.volume_slice(costMap3D, plane_orientation='y_axes', plane_opacity = 0.1, transparent = True)
mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)

if path3D.ndim == 2:
	fig2 = mlab.figure()
	mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
	if path.ndim == 1:
	    mlab.plot3d(path[0]/res, path[1]/res, (1+0.625)/resz*np.ones(path.size), color=(1,1,1), tube_radius = 1)
	else:
	    mlab.plot3d(path[:,0]/res, path[:,1]/res, (1+0.625)/resz*np.ones(len(path)), color=(1,1,1), tube_radius = 1)
	mlab.plot3d(path3D[:,0]/res, path3D[:,1]/res, path3D[:,2]/resz, color=(0.3,0.3,0.5), tube_radius = 1)
	mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)
	plt_vs = mlab.volume_slice(costMap3D, plane_orientation='y_axes', plane_opacity = 0.1, transparent = True,slice_index=80)     

f = mlab.gcf();
f.scene._lift();
duration = 10


def make_frame(t):
    mlab.clf()
    #vs.remove()
    mlab.contour3d(box, contours = 5, opacity = 0.05, transparent = True)
    mlab.plot3d(path[:,0]/res, path[:,1]/res, (1+0.625)/resz*np.ones(len(path)), color=(1,1,1), tube_radius = 1)
    mlab.plot3d(path3D[:,0]/res, path3D[:,1]/res, path3D[:,2]/resz, color=(0.3,0.3,0.5), tube_radius = 1)
    mlab.quiver3d(np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]), scale_factor = 20)
    mlab.volume_slice(costMap3D, plane_orientation='y_axes', plane_opacity = 0.1, transparent = True,slice_index=10*t)     
    #mlab.view(azimuth = -110, elevation = 50, distance = 20)
    #plt_vs.slice_index=((int)(t/duration))*100
    return mlab.screenshot(antialiased=True)


#animation = mpy.VideoClip(make_frame, duration=duration)
#animation.write_gif("tunnelcost.gif", fps=12, program='imageio',opt = 'nq')

mlab.show()
