import matplotlib.pyplot as plt

def get_obstacles(filename):
	'return list of tuples of three xy coordinates'
	file = open(filename,'r')
	triangles = []
	for line in file:
		line = line[:-1]
		equations = []
		for eqn in line.split(':'):
			eqn = eqn[1:-1]
			constants = [float(c) for c in eqn.split(',')]
			equations.append(constants)
		intersections = []
		for i in range(3):
			a,b,c = equations[i]
			d,e,f = equations[(i+1)%3]
			if e == 0:
				x = -f/d
				y = -(a/b)*x-(c/b)
			elif b == 0:
				x = -c/a
				y = -(d/e)*x-(f/e)
			elif d == a:
				y = (f-c)/(b-e)
				x = -(b*y+c)/a
			else:
				x = (c/b-f/e)/(d/e-a/b)
				y = -(a/b)*x-(c/b)

			intersections.append((x,y))
		triangles.append(intersections)
	return triangles

def plot_obstacles(triangles):
	fig, ax = plt.subplots()
	plt.xlim(-10,10)
	plt.ylim(-10,10)
	for tri in triangles:
		t2 = plt.Polygon(tri, color='black')
		plt.gca().add_patch(t2)

	plt.show()



if __name__ == "__main__":
	triangles = get_obstacles('Environment.txt')
	plot_obstacles(triangles)
