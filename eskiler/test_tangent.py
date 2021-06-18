import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')
theta = np.linspace(0, 4 * np.pi, 100)
z = np.linspace(0, 2*5, 100)
r = (z**2 + 1 )
x = (r * np.sin(theta)) / 15 
y = (r * np.cos(theta)) / 15
for i in range (len(x)-1):
    print(x[i],y[i],z[i])

ax.plot(x, y, z, label='parametric curve')
ax.legend()

plt.show()