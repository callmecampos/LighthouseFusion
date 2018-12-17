from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
fig = plt.figure()
#axis = plt.axes(projection='3d')

#ax.set_xlim(-1.0, 1.0)
#ax.set_ylim(-1.0, 1.0)
#ax.set_zlim(-1.0, 1.0)
time=np.loadtxt("C:\\Users\\more3D\\Desktop\\acceldiv.txt").T[0]
xyz = np.loadtxt("C:\\Users\\more3D\\Desktop\\acceldiv.txt").T[1:4]
axyz = np.loadtxt("C:\\Users\\more3D\\Desktop\\acceldiv.txt").T[4:7]

ax, ay, az = axyz[0], axyz[1], axyz[2]

print(xyz)

# Data for three-dimensional scattered points
xdata = xyz[0]; print(xdata)
ydata = xyz[1]; print(ydata)
zdata = xyz[2]; print(zdata)
# axis.scatter3D(xdata, ydata, zdata)

print("varx: {}, vary: {}, varz: {}".format(np.var(ax, ddof=1), np.var(ay, ddof=1), np.var(az, ddof=1)))
print("start: {}, end: {}, dist: {}".format(xyz.T[0], xyz.T[len(xyz.T)-1],
                                            np.linalg.norm(xyz.T[0].T - xyz.T[len(xyz.T)-1].T)))

# plt.axis('equal')
# plt.show()
# plt.plot(time/60000,xdata,time/60000,ydata, time/60000, zdata)
# fig.suptitle('XYZ Position vs Time', fontsize=20)
# plt.xlabel('Time (Minutes)', fontsize=18)
# plt.ylabel('Position (Meters)', fontsize=16)
# plt.legend(['x-axis','y-axis','z-axis'])

plt.plot(time/60000,ax,time/60000,ay, time/60000, az)
fig.suptitle('Acceleration vs Time', fontsize=20)
plt.xlabel('Time (Minutes)', fontsize=18)
plt.ylabel('Acceleration (m/s^2)', fontsize=16)
plt.legend(['x-axis','y-axis','z-axis'])
#plt.legend((line1, line2, line3),('test','test2','test3'))
print(stats.describe(ax))
print(stats.describe(ay))
print(stats.describe(az))
plt.show()
fig.savefig('IMUDrift.jpg')