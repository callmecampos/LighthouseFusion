from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection='3d')

#ax.set_xlim(-1.0, 1.0)
#ax.set_ylim(-1.0, 1.0)
#ax.set_zlim(-1.0, 1.0)

xyz = np.loadtxt('~/Desktop/looptest1/pose_read.txt').T

# Data for three-dimensional scattered points
xdata = xyz[0]
ydata = xyz[1]
zdata = xyz[2]
ax.scatter3D(xdata, ydata, zdata)

print("varx: {}, vary: {}, varz: {}".format(np.var(xdata), np.var(ydata), np.var(zdata)))
print("start: {}, end: {}, dist: {}".format(xyz.T[0], xyz.T[len(xyz.T)-1],
                                            np.linalg.norm(xyz.T[0].T - xyz.T[len(xyz.T)-1].T)))

plt.axis('equal')
plt.show()
