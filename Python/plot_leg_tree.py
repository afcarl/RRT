import matplotlib.pyplot as plt
import string
import math
import numpy as np

# plt.ion()
filename_leg = 'leg_tree.txt'
file_leg = open(filename_leg,'r')
filename_rand = 'leg_tree_rand.txt'
file_rand = open(filename_rand,'r')
th1 = []
th2 = []
rth1 = []
rth2 = []


for line in file_leg:	# each line represents a series of x,y points in a path from two nodes
	line = line[1:-2]
	angles = [float(w) for w in line.split(',')]
	th1.append(angles[0])
	th2.append(angles[1])

for line in file_rand:	# each line represents a series of x,y points in a path from two nodes
	line = line[1:-2]
	angles = [float(w) for w in line.split(',')]
	rth1.append(angles[0])
	rth2.append(angles[1])


fig, ax = plt.subplots()
# plt.plot(path_x,path_y,'g-')
# plt.plot(toe_x_vec,toe_y_vec,'r-')
plt.plot(th1,th2,'k.')
plt.plot(rth1,rth2,'r.')
# for a in range(len(knee_x_vec)):
	# plt.plot([0,knee_x_vec[a]],[0,knee_y_vec[a]],'k-')
	# plt.plot([knee_x_vec[a],toe_x_vec[a]],[knee_y_vec[a],toe_y_vec[a]],'k-')
plt.xlim(0,2*math.pi)
plt.ylim(0,2*math.pi)
# ax.yaxis.set_visible(False)
# ax.xaxis.set_visible(False)
plt.show()

