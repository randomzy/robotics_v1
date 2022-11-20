#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from numpy import genfromtxt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection

points = genfromtxt('points.csv', delimiter=',')
x = points[:,0]
y = points[:,1]
z = points[:,2]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for a,b in zip(points[0:], points[1:]):
    ax.plot([a[0],b[0]],[a[1],b[1]],[a[2],b[2]])

ax.set_xlim(min(x)-10, max(x)+10)
ax.set_ylim(min(y)-10, max(y)+10)
ax.set_zlim(min(z), max(z))

ax.set_xlabel('X', color = 'r')
ax.set_ylabel('Y', color = 'r')
ax.set_zlabel('Z', color = 'r')

colormap = plt.cm.gist_ncar #nipy_spectral, Set1,Paired   
colors = [colormap(i) for i in np.linspace(0, 0.8, len(ax.lines))]
for i,j in enumerate(ax.lines):
    j.set_color(colors[i])

plt.show()
