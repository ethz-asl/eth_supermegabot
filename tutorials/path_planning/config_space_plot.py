import numpy as np
import matplotlib.pyplot as plt
import polygon_tools as poly
import robot_tools
from matplotlib.patches import Polygon as PlotPolygon
from matplotlib.collections import PatchCollection
from skimage import measure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import copy
plt.rc('font',**{'family':'serif','sans-serif':['Computer Modern Roman']})
plt.rc('text', usetex=True)

nx = 101
num_obstacles = 5
n_obs_samples = 5
obs_std = 0.1
np.random.seed(5)

# Generate obstacles (random points then convex hull)
obs_centres = [poly.Point(*np.random.uniform(size=2)) for i in range(num_obstacles)]
obstacles = []
for pc in obs_centres:
    px, py = np.random.normal(pc, obs_std, size=(n_obs_samples, 2)).T
    px, py = np.clip(px, 0.0, 1.0), np.clip(py, 0.0, 1.0)
    p = poly.PointList([poly.Point(x, y) for x, y in zip(px, py)])
    p = poly.convex_hull(p)
    obstacles.append(p)

# Get some random points and see if they're in the obstacles:
in_obs, out_obs = poly.PointList([]), poly.PointList([])
for i in range(200):
    p = poly.Point(*np.random.uniform(size=2))
    collision = False
    for o in obstacles:
        if o.point_inside(p):
            collision = True
            break
    if collision:
        in_obs.append(p)
    else:
        out_obs.append(p)

f1, a1 = plt.subplots()
h_obs = []
for o in obstacles:
    h_obs.append(PlotPolygon(o, color='lightgrey', zorder=1))
c_obs = PatchCollection(h_obs)
a1.add_collection(c_obs)
a1.scatter(*zip(*in_obs), color='r', marker='x')
a1.scatter(*zip(*out_obs), color='g', marker='.')
print "Intersect: {0}".format(obstacles[0].intersect(obstacles[1]))


# Now try robot poses:
# robo_footprint = poly.PointList([poly.Point(0.05, 0.0), poly.Point(-0.03, 0.03), poly.Point(-0.03, -0.03)])
robo_footprint = poly.PointList([poly.Point(0.1, 0.01), poly.Point(-0.1, 0.01), poly.Point(-0.1, -0.01), poly.Point(0.1, -0.01)])
robo = robot_tools.Robot2D(footprint=robo_footprint)
a1.add_artist(PlotPolygon(robo.get_current_polygon(), facecolor='r'))

robo.set_position((0.25, 0.38))
robo.get_current_polygon().intersect(obstacles[-1])

x, y, h = np.linspace(0, 1, 51), np.linspace(0, 1, 51), np.linspace(0, np.pi, 41)
v = np.zeros((len(x), len(y), len(h)))
for i,xi in enumerate(x):
    for j, yj in enumerate(y):
        robo.set_position((xi, yj))
        for k, hk in enumerate(h):
            in_obs = 0.0
            robo.set_heading(hk)
            fp = robo.get_current_polygon()
            for o in obstacles:
                if fp.intersect(o):
                    in_obs = 1.0
                    break
            v[i, j, k] = in_obs

verts, faces, normals, values = measure.marching_cubes(v, spacing=(x[1]-x[0], y[1]-y[0], (h[1]-h[0])*180/np.pi))
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')
ax.plot_trisurf(verts[:, 0], verts[:,1], faces, verts[:, 2],
                cmap='Spectral', lw=1)
ax.set_xlim(0, x[-1])  # a = 6 (times two for 2nd ellipsoid)
ax.set_ylim(0, y[-1])  # b = 10
ax.set_zlim(0, h[-1]*180/np.pi)  # c = 16
ax.set_xlabel(r'$x_c$')
ax.set_ylabel(r'$y_c$')
ax.set_zlabel(r"$\theta (^{\circ})$")


robo.set_position([0.1, 0.1])
f2, a2 = plt.subplots(2, 2)
for i, ax in enumerate(a2.flat):
    dex = int(i*0.25*(len(h)-1))
    ax.matshow(v[:, :, dex].transpose(), origin='lower', extent=[0,1,0,1], cmap='Greys')
    ax.add_collection(PatchCollection(copy.copy(h_obs)))
    robo.set_heading(h[dex])
    ax.add_artist(PlotPolygon(robo.get_current_polygon(), facecolor='r'))
    ax.plot(*robo.position, color='g', marker='x')
    ax.set_title(r"$\theta = {0}$".format(h[dex]*180/np.pi))
    ax.tick_params(top=0, left=0)

# random.seed(1)
# true_g = fm_graphtools.CostmapGrid(gridsize[0], gridsize[1])
# true_g.obstacles = fm_plottools.generate_obstacles(gridsize[0], gridsize[1], nobs, obs_size)
#
# f1, a1 = fm_plottools.init_fig(true_g)
# fm_plottools.draw_grid(a1, true_g)

plt.show()

