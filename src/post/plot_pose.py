from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection='3d')

#ax.set_xlim(-1.0, 1.0)
#ax.set_ylim(-1.0, 1.0)
#ax.set_zlim(-1.0, 1.0)

xyzg = np.loadtxt('~/lighthouse-slam/pilot/looptest1/pose_lh.txt').T
xyze = np.loadtxt('~/lighthouse-slam/pilot/looptest1/pose_ef.txt').T
# Data for three-dimensional scattered points
xg, yg, zg = xyzg[1], xyzg[2], xyzg[3]
xe, ye, ze = xyze[1], xyze[2], xyze[3]
ax.scatter3D(xg, yg, zg)
ax.scatter3D(xe, ye, ze)

print("varx: {}, vary: {}, varz: {}".format(np.var(xdata), np.var(ydata), np.var(zdata)))
print("start: {}, end: {}, dist: {}".format(xyz.T[0], xyz.T[len(xyz.T)-1],
                                            np.linalg.norm(xyz.T[0].T - xyz.T[len(xyz.T)-1].T)))

plt.axis('equal')
plt.show()
