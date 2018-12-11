from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = plt.axes(projection='3d')

xyz = np.loadtxt('poses.txt').T

# Data for three-dimensional scattered points
zdata = xyz[2]
xdata = xyz[0]
ydata = xyz[1]
ax.scatter3D(xdata, ydata, zdata, c=zdata)

plt.show()
