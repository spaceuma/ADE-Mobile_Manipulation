import os 
import numpy as np 
import matplotlib.pyplot as plt 
from pytransform3d.urdf import UrdfTransformManager 
import math

sizes = np.loadtxt(open("../data/planner/reachabilityDistancesAccess.txt", 'r'), max_rows = 1)
#sizes = np.loadtxt(open("../data/planner/reachabilityMap.txt", 'r'), max_rows = 1)
#sizes = np.loadtxt(open("../data/planner/reachabilityMap_Coupled.txt", 'r'), max_rows = 1)

resolutions = np.loadtxt(open("../data/planner/reachabilityDistancesAccess.txt", 'r'), skiprows = 1, max_rows = 1)
#resolutions = np.loadtxt(open("../data/planner/reachabilityMap.txt", 'r'), skiprows = 1, max_rows = 1)
#resolutions = np.loadtxt(open("../data/planner/reachabilityMap_Coupled.txt", 'r'), skiprows = 1, max_rows = 1)

minValues = np.loadtxt(open("../data/planner/reachabilityDistancesAccess.txt", 'r'), skiprows = 2, max_rows = 1)
#minValues = np.loadtxt(open("../data/planner/reachabilityMap.txt", 'r'), skiprows = 2, max_rows = 1)
#minValues = np.loadtxt(open("../data/planner/reachabilityMap_Coupled.txt", 'r'), skiprows = 2, max_rows = 1)

reachabilityMap2D = np.loadtxt(open("../data/planner/reachabilityDistancesAccess.txt", 'r'), skiprows = 3)
#reachabilityMap2D = np.loadtxt(open("../data/planner/reachabilityMap.txt", 'r'), skiprows = 3)
#reachabilityMap2D = np.loadtxt( open("../data/planner/reachabilityMap_Coupled.txt", 'r'), skiprows = 3)

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
  for j in range(0, ysize * zsize): 
    reachabilityMap3D[i][k][c] = reachabilityMap2D[i][j] 
    c += 1 
    if c > zsize - 1: 
      c = 0 
      k += 1

BASE_DIR = "/home/ares/ADE-Mobile_Manipulation/data/planner/" 
if not os.path.exists(BASE_DIR): 
  BASE_DIR = os.path.join("..", BASE_DIR)

tm = UrdfTransformManager() 
with open(BASE_DIR + "sherpa_plotting.urdf", "r") as f:
 tm.load_urdf(f.read(), mesh_path = BASE_DIR)

tm.set_joint("beta_front_left", - 0.46) 
tm.set_joint("beta1_fake_front_left", 0.0 * math.pi / 180.0) 
tm.set_joint("beta2_fake_front_left", - 0.46)

tm.set_joint("beta_front_right", - 0.46) 
tm.set_joint("beta1_fake_front_right", 0.0 * math.pi / 180.0) 
tm.set_joint("beta2_fake_front_right", - 0.46)

tm.set_joint("beta_rear_left", - 0.46) 
tm.set_joint("beta1_fake_rear_left", 0.0 * math.pi / 180.0) 
tm.set_joint("beta2_fake_rear_left", - 0.46)

tm.set_joint("beta_rear_right", - 0.46) 
tm.set_joint("beta1_fake_rear_right", 0.0 * math.pi / 180.0) 
tm.set_joint("beta2_fake_rear_right", - 0.46)

tm.set_joint("gamma_front_left", 0.61)
#tm.set_joint("gamma1_fake_front_left", 0.61)
tm.set_joint("gamma2_fake_front_left", 0.61)

tm.set_joint("gamma_front_right", 0.61)
#tm.set_joint("gamma1_fake_front_right", 0.61)
tm.set_joint("gamma2_fake_front_right", 0.61)

tm.set_joint("gamma_rear_left", 0.61)
#tm.set_joint("gamma1_fake_rear_left", 0.61)
tm.set_joint("gamma2_fake_rear_left", 0.61)

tm.set_joint("gamma_rear_right", 0.61)
#tm.set_joint("gamma1_fake_rear_right", 0.61)
tm.set_joint("gamma2_fake_rear_right", 0.61)

fig = plt.figure() 
ax = fig.gca(projection = '3d')

#plotted = np.zeros([np.size(reachabilityMap3D, 0), np.size(reachabilityMap3D, 1), np.size(reachabilityMap3D, 2)])
#prevReach = 0
#for i in range(0, xsize):
# for j in range(0, ysize):
#   for k in range(0, zsize):
#     if (reachabilityMap3D[i][j][k] == 1) :
#       reachabilityMap3D[i][j][k] = 2
#     if (reachabilityMap3D[i][j][k] != prevReach) :
#       prevReach = reachabilityMap3D[i][j][k]
#       c = 'r'
#
#       if (not(plotted[i][j][k])) :
#         plotted[i][j][k] = 1
#         ax.scatter(i *resXY + minValues[0], j * resXY + minValues[1], k * resZ + minValues[2], c = c, marker = 'o')
#
#prevReach = 0
#for i in range(0, xsize):
# for k in range(0, zsize):
#   for j in range(0, ysize):
#     if (reachabilityMap3D[i][j][k] == 1) :
#       reachabilityMap3D[i][j][k] = 2
#     if (reachabilityMap3D[i][j][k] != prevReach) :
#       prevReach = reachabilityMap3D[i][j][k]
#       c = 'r'
#
#       if (not(plotted[i][j][k])) :
#         plotted[i][j][k] = 1
#         ax.scatter(i *resXY + minValues[0], j * resXY + minValues[1], k * resZ + minValues[2], c = c, marker = 'o')
#
#prevReach = 0
#for j in range(0, ysize):
# for k in range(0, zsize):
#   for i in range(0, xsize):
#     if (reachabilityMap3D[i][j][k] == 1) :
#       reachabilityMap3D[i][j][k] = 2
#     if (reachabilityMap3D[i][j][k] != prevReach) :
#       prevReach = reachabilityMap3D[i][j][k]
#       c = 'r'
#
#       if (not(plotted[i][j][k])) :
#         plotted[i][j][k] = 1
#         ax.scatter(i *resXY + minValues[0], j * resXY + minValues[1], k * resZ + minValues[2], c = c, marker = 'o')

xs = resXY * range(0, xsize + 1) + minValues[0] 
ys = resXY * range(0, ysize + 1) + minValues[1] 
zs = resZ * range(0, zsize + 1) + minValues[2]

X, Y, Z = np.meshgrid(xs, ys, zs)
#Z = np.zeros([xsize, ysize])
#for k in range(0, zsize):
# for i in range(0, xsize):
#   for j in range(0, ysize):
#     Z[i, j] = reachabilityMap3D[j, i, k] > 1.99
#     if (Z[i, j] < 1) :
#       Z[i, j] = np.inf
# ax.plot_surface(X, Y, k *resZ *Z + minValues[2], color = [1, 0, 0], alpha = 0.1, shade = 1)
ax.voxels(Y, X, Z, reachabilityMap3D, alpha = 0.1, shade = 0) 
ax.set_xlabel("X-axis (m)") 
ax.set_ylabel("Y-axis (m)") 
ax.set_zlabel("Z-axis (m)")

tm.plot_visuals("body", ax = ax)

plt.show()
