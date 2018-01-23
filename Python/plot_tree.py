import matplotlib.pyplot as plt
import string
import numpy as np
import Environment

# plt.ion()
filename_tree = 'tree.txt'
file_tree = open(filename_tree,'r')
x_values = []
y_values = []
directions = []
for line in file_tree:
	# each line represents a series of x,y points in a path from two nodes
	points = [word for word in line.split(' ')]
	is_forward = points.pop(0) 					# 1 for forward and 0 for reverse
	points = points[:-1]						 # to remove end line character
	points = [[float(a) for a in p[1:-1].split(',')] for p in points]
	points = map(list, zip(*points))			# converts to list of x data then list of y data	
	x_values.append(points[0])
	y_values.append(points[1])
	directions.append(int(is_forward))



fig, ax = plt.subplots()

# obstacles
ax.axvspan(6, 7, ymin=0.0, ymax=0.7, alpha=0.8, color='black')
ax.axvspan(3, 4, ymin=0.4, ymax=1.0, alpha=0.8, color='black')
# t2 = plt.Polygon([[2,2],[4,3],[6,1]], color='black')
# plt.gca().add_patch(t2)

plt.plot(1,1,'bo',markersize=10)
# plt.plot(9,9,'gs',markersize=10)
line1, = plt.plot([],[],'bx')
plt.xlim(0,10)
plt.ylim(0,10)
ax.yaxis.set_visible(False)
ax.xaxis.set_visible(False)
# for a in range(len(x_values)):
# 	color = 'g-' if directions[a] == 1 else 'r-'
# 	if directions[a] == 1:
# 		plt.plot(x_values[a],y_values[a],color)
# for a in range(len(rand_x_values)):
# 	plt.plot(rand_x_values[a],rand_y_values[a],'b*')
# plt.show()


for a in range(len(x_values)):
	color = 'g-' if directions[a] == 1 else 'r-'
	# line1.set_data(rand_x_values[0],rand_y_values[0])
	plt.plot(x_values[a],y_values[a],color)
	plt.draw()
	plt.pause(0.001)
plt.show()

# each link should encode direction and curve
