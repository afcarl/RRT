import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
import string
import math
import numpy as np


filename_tree = 'car_tree.txt'
file_tree = open(filename_tree,'r')
tree = []
for line in file_tree:	# each line represents a series of x,y points in a path from two nodes
	line = line[1:-3]
	x = []
	y = []
	lst = [float(w) for w in line.split(',')]
	for a in range(len(lst)):
		if a % 3 == 0:
			x.append(lst[a])
		if a % 3 == 1:
			y.append(lst[a])
		if a % 3 == 2:
			th = lst[a]
	angle = math.atan((y[-1]-y[-2])/(x[-1]-x[-2]))
	if math.fmod(abs(th-angle),2*math.pi) < math.pi/2:
		direction = 1
	else:
		direction = 1
	tree.append([x,y,direction])

fig = plt.figure()
ax = fig.add_subplot(111)
x_tree,y_tree,th_tree = zip(*tree)
L = 0.2
for t in tree:
	if t[2] == 1:
		color = 'b-'
	else:
		color = 'r-'
	plt.plot(t[0],t[1],color)

############################

def make_car(x,y,angle,width=0.3,length=0.5):
	def rotate(vec,a):
		ret = [0,0]
		ret[0] = vec[0]*math.cos(a)-vec[1]*math.sin(a)
		ret[1] = vec[0]*math.sin(a)+vec[1]*math.cos(a)
		return np.array(ret)

	l_axis = 0.9*length
	# top left
	TL = [-(length-l_axis)/2.0,width/2.0]
	TL = rotate(TL,angle)+np.array([x,y])
	TR = [l_axis+(length-l_axis)/2.0,width/2.0]
	TR = rotate(TR,angle)+np.array([x,y])
	BL = [-(length-l_axis)/2.0,-width/2.0]
	BL = rotate(BL,angle)+np.array([x,y])
	BR = [l_axis+(length-l_axis)/2.0,-width/2.0]
	BR = rotate(BR,angle)+np.array([x,y])
	return [TL,TR,BR,BL,TL]

filename_path = 'car_path.txt'
file_path = open(filename_path,'r')
path = []
for line in file_path:	# each line represents a series of x,y points in a path from two nodes
	line = line[1:-2]
	new = [float(w) for w in line.split(',')]
	path.append(new)
t_path,x_path,y_path,th_path = zip(*path)


codes = [Path.MOVETO,Path.LINETO,Path.LINETO,Path.LINETO,Path.CLOSEPOLY]

delta_t = t_path[1]-t_path[0]
plt.plot(x_path,y_path,'g-')

for i in range(len(path)):
	verts = make_car(x_path[i],y_path[i],th_path[i])
	path = Path(verts, codes)
	if i!=0:
		patch.remove()
	patch = patches.PathPatch(path, facecolor='black', lw=2)
	ax.add_patch(patch)
	plt.draw()
	plt.pause(0.001)



