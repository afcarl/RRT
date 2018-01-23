import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

import numpy as np
import math
import time

verts = np.array([[0.,0.],[0.,1.],[1.,1.],[1.,0.],[0.,0.]])

codes = [Path.MOVETO,
         Path.LINETO,
         Path.LINETO,
         Path.LINETO,
         Path.CLOSEPOLY,
         ]

path = Path(verts, codes)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim(-2,2)
ax.set_ylim(-2,2)
plt.show(block=False)
i=0.0
while i<100:
#for i in range(100):
    ax.clear()
    x=math.sin(i)
    y=math.cos(i)

    # print x, y

    verts2=np.copy(verts)
    for j in range(len(verts)):
        verts2[j]=verts[j]+np.array([x,y])
    path = Path(verts2, codes)
    patch = patches.PathPatch(path, facecolor='orange', lw=2)
    ax.add_patch(patch)
    plt.draw()
    plt.pause(0.01)
    i+=0.1