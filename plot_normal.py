import matplotlib.pyplot as plt
import string
import math
import numpy as np

# plt.ion()
filename_tree = 'normal_tree.txt'
file_tree = open(filename_tree,'r')
filename_path = 'normal_path.txt'
file_path = open(filename_path,'r')
path = []
tree = []


for line in file_tree:	# each line represents a series of x,y points in a path from two nodes
	line = line[1:-2]
	x1,y1,x2,y2= [float(w) for w in line.split(',')]
	tree.append([[x1,x2],[y1,y2]])

for line in file_path:	# each line represents a series of x,y points in a path from two nodes
	line = line[1:-2]
	new = [float(w) for w in line.split(',')]
	path.append(new)



plt.figure()

for p in tree:
	plt.plot(p[0],p[1],'k-')
x_path,y_path = zip(*path)
plt.plot(x_path,y_path,'r-')
plt.show()