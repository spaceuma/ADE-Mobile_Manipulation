
import sys
import numpy as np
import matplotlib.pyplot as plt
#import scipy.ndimage
from mayavi import mlab
import plotly.graph_objects as go

sizes = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), max_rows=1)
resolutions = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), skiprows = 1, max_rows=1)
minValues = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), skiprows = 2, max_rows=1)
reachabilityMap2D = np.loadtxt(open("../../data/planner/reachabilityDistances_Coupled.txt",'r'), skiprows=3)

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


stopx = xsize*resXY+minValues[0]
stopy = ysize*resXY+minValues[1]
stopz = zsize*resZ+minValues[2]
complexSizex = complex(0,xsize)
complexSizey = complex(0,ysize)
complexSizez = complex(0,zsize)

x = np.linspace(minValues[0],stopx,xsize)
y = np.linspace(minValues[1],stopy,ysize)
z = np.linspace(minValues[2],stopz,zsize)

X, Y, Z = np.mgrid[minValues[0]:stopx:complexSizex,minValues[1]:stopy:complexSizey,minValues[2]:stopz:complexSizez]


#normReachability = reachabilityMap3D/np.max(reachabilityMap3D)

#fig1 = mlab.figure()
#mlab.contour3d(normReachability, contours = 20, opacity = 0.2, transparent = False)
#mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([20]), np.array([0]), np.array([0]), scale_factor = 0.3, color = (1,0,0))
#mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([0]), np.array([20]), np.array([0]), scale_factor = 0.3, color = (0,1,0))
#mlab.quiver3d(np.array([xsize/2]), np.array([ysize/2]), np.array([zsize/2]), np.array([0]), np.array([0]), np.array([20]), scale_factor = 0.3, color = (0,0,1))
#mlab.volume_slice(normReachability, plane_orientation='x_axes', opacity = 0, plane_opacity = 0, transparent = True)
#mlab.show()


volume = np.zeros(reachabilityMap3D.shape)
for i in range(0,xsize):
  for j in range(0,ysize):
    for k in range(0,zsize):
      if reachabilityMap3D[i,j,k] != 0:
        volume[j,i,k] = 1+0.3/reachabilityMap3D[i,j,k]
      else:
        volume[j,i,k] = 999
r, c = volume[0].shape

# Define frames
import plotly.graph_objects as go
nb_frames = zsize

fig = go.Figure(frames=[go.Frame(data=go.Surface(
    x = x,
    y = y,
    z=(stopz - k * resZ) * np.ones((r, c)),
    surfacecolor=volume[:,:,zsize-1 - k],
    cmin=1.4, cmax=6,
    colorscale='jet_r',
    opacity=0.8, # needs to be small to see through all surfaces
    showscale = True,
    ),
    name=str(k) # you need to name the frame for the animation to behave properly

    )
    for k in range(nb_frames)])

# Add data to be displayed before animation starts
fig.add_trace(go.Surface(
    x = x,
    y = y,
    z=stopz * np.ones((r, c)),
    surfacecolor=volume[:,:,zsize-1],
    cmin=1.4, cmax=6,
    colorscale='jet_r',
    opacity=0.8, # needs to be small to see through all surfaces
    showscale = True,
    name = "Distance",
    ))

# Add complete volume representation
fig.add_trace(go.Volume(
    x = X.flatten(),
    y=Y.flatten(),
    z=Z.flatten(),
    value=reachabilityMap3D.flatten(),
    colorscale=[[0, 'rgb(105,176,250)'],[1,'rgb(105,176,250)']],
    isomin=0.001,
    isomax=0.99,
    opacity=0.1, # needs to be small to see through all surfaces
    showscale = False,
    surface_count=2, # needs to be a large number for good volume rendering
    ))


def frame_args(duration):
    return {
            "frame": {"duration": duration},
            "mode": "immediate",
            "fromcurrent": True,
            "transition": {"duration": duration, "easing": "linear"},
        }

sliders = [
            {
                "pad": {"b": 10, "t": 60},
                "len": 0.9,
                "x": 0.1,
                "y": 0,
                "steps": [
                    {
                        "args": [[f.name], frame_args(0)],
                        "label": str(k),
                        "method": "animate",
                    }
                    for k, f in enumerate(fig.frames)
                ],
            }
        ]

# Layout
fig.update_layout(
         title='Slices in volumetric data',
         width=800,
         height=800,
         scene=dict(
                    zaxis=dict(range=[minValues[2], stopz], autorange=False),
                    aspectratio=dict(x=1, y=1, z=1),
                    xaxis_title="X-axis (m)",
                    yaxis_title="Y-axis (m)",
                    zaxis_title="Z-axis (m)",
                    ),
         updatemenus = [
            {
                "buttons": [
                    {
                        "args": [None, frame_args(50)],
                        "label": "&#9654;", # play symbol
                        "method": "animate",
                    },
                    {
                        "args": [[None], frame_args(0)],
                        "label": "&#9724;", # pause symbol
                        "method": "animate",
                    },
                ],
                "direction": "left",
                "pad": {"r": 10, "t": 70},
                "type": "buttons",
                "x": 0.1,
                "y": 0,
            }
         ],
         sliders=sliders
)

fig.show()
