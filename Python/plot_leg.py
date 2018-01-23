import matplotlib.pyplot as plt
import string
import numpy as np

# plt.ion()
filename_leg = 'leg.txt'
file_leg = open(filename_leg,'r')
knee_x_vec = []
knee_y_vec = []
toe_x_vec = []
toe_y_vec = []
time_vec = []

count = 0
for line in file_leg:	# each line represents a series of x,y points in a path from two nodes
	count += 1
	line = line[1:-2]
	angles = [float(w) for w in line.split(',')]
	time = angles.pop(0)
	time_vec.append(time)
	knee_x_vec.append(np.sin(angles[0]))
	knee_y_vec.append(-np.cos(angles[0]))
	toe_x_vec.append(np.sin(angles[0])+np.sin(angles[1]))
	toe_y_vec.append(-np.cos(angles[0])-np.cos(angles[1]))

# filename_path = 'path.txt'
# file_path = open(filename_path,'r')
# path_x = []
# path_y = []
# for line in file_path:
# 	line = line[1:-2]
# 	x,y = [float(w) for w in line.split(',')]
# 	path_x.append(x)
# 	path_y.append(y)

fig, ax = plt.subplots()
# plt.plot(path_x,path_y,'g-')
# plt.plot(toe_x_vec,toe_y_vec,'r-')
link1, = plt.plot([],[],'k-')
link2, = plt.plot([],[],'k-')
plt.xlim(-3,3)
plt.ylim(-3,3)
ax.yaxis.set_visible(False)
ax.xaxis.set_visible(False)

for a in range(len(knee_x_vec)):
	link1.set_data([0,knee_x_vec[a]],[0,knee_y_vec[a]])
	link2.set_data([knee_x_vec[a],toe_x_vec[a]],[knee_y_vec[a],toe_y_vec[a]])
	plt.draw()
	delta_t = time_vec[a+1]-time_vec[a] if a < (len(knee_x_vec)-1) else 0.001
	plt.pause(max(delta_t,0.001))


# # # each link should encode direction and curve
