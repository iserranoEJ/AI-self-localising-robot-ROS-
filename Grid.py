from __future__ import print_function



# Node class used to encode vertices in the A* search.
class Node:
	
	# Constructor. Sets x, y values, and defaults remaining attributes.
	def __init__(self, grid, x, y):
	
		self.pathCost = 0
		self.hDistance = 0
		self.accessible = True
		self.predecessor = None
		self.x = x
		self.y = y
		self.grid = grid

	
	# Computes the f(n) value for this node.
	def fScore(self):
		return self.pathCost + self.hDistance
	
	# Calculates the unique hash value of this node
	# as its position in the grid (interpreted as
	# a linear array). Used for perfect hashing 
	# in the A* priority queue.
	def __hash__(self):
		return self.x * self.grid.gridDim + self.x
	
	# Used to determine whether two nodes are equal. Required
	# in order to use nodes as keys within the A* priority queue.
	def __eq__(self, other):
		return (self.x, self.y) == (other.x, other.y)

	# Less than operator. Used as a comparator in the A* priority
	# queue
	def __lt__(self, other):
		return self.fScore() < other.fScore()
	

# Grid class used to encode the graph for the A* search.
class Grid:

	# Constructor. Takes grid dimensions as input.
	def __init__(self, num_squares, sidelength, start_x, start_y):
	    
	    # dimension of the grid (number of squares)
		self.gridDim = num_squares
		# physical size of each square
		self.boxDim = float(sidelength) / float(num_squares)
		# coordinates of bottom left point
		self.start_x = start_x
		self.start_y = start_y
		# and the actual array of Nodes
		self.grid = [[Node(self, y, x) for x in range(num_squares)] for y in range(num_squares)]
	
	# Returns a reference to the node at the given coordinates.
	def get(self, x, y):
		return self.grid[x][y]

	
	# Sets the given grid cell to be inaccessible. Takes cartesian coordinates x and y.
	def setInaccessible(self, x, y):
		x -= self.start_x
		y -= self.start_y
		self.grid[int(x / self.boxDim)][int(y / self.boxDim)].accessible = False

	# Returns true if the given cell location is in bounds.
	def inbounds(self, x, y):
		return x >= 0 and x < gridDim and y >= 0 and y < gridDim 

	# Returns true if the given grid cell is accessible.
	def accessible(self, x, y):
		return self.grid[x][y].accessible


	# Returns an unordered list of all unoccupied grid cells that are reachable from the given one.
	def reachable(self, node):

		x = node.x
		y = node.y
		results = []

        # strategy is to check if each neighbouring square is accessible

        # four orthogonal directions
		if x < self.gridDim - 1 and self.grid[x + 1][y].accessible: 
			results.append(self.grid[x + 1][y])

		if x > 0 and self.grid[x - 1][y].accessible:
			 results.append(self.grid[x - 1][y])

		if y < self.gridDim - 1 and self.grid[x][y + 1].accessible: 
			results.append(self.grid[x][y + 1])

		if y > 0 and self.grid[x][y - 1].accessible: 
			results.append(self.grid[x][y - 1])

        # and four diagonals (here, you can't cut across an inaccessible square, so extra checks
        # are needed).
		if y < self.gridDim - 1 and x > 0 and self.accessible(x - 1, y) \
		        and self.accessible(x - 1, y + 1) \
		        and self.accessible(x, y + 1):
			results.append(self.grid[x - 1][y + 1])

		if y < self.gridDim - 1 and x < self.gridDim - 1 and self.accessible(x+1, y) \
		        and self.accessible(x + 1, y + 1) \
		        and self.accessible(x, y + 1):
			results.append(self.grid[x + 1][y + 1])

		if y > 0 and x < self.gridDim - 1 and self.accessible(x+1, y) \
		        and self.accessible(x + 1, y - 1) \
		        and self.accessible(x, y - 1):
			results.append(self.grid[x + 1][y - 1])

		if y > 0 and x > 0 and self.accessible(x-1, y) \
		        and self.accessible(x - 1, y - 1) \
		        and self.accessible(x, y - 1):
			results.append(self.grid[x - 1][y - 1])

		return results
		
	
	# Print the grid.
	def printGrid(self):
		
		print(" ", end=" ")
		for y in range(len(self.grid)):
			print(y, end=" "),
		print("")
		for x in range(len(self.grid)):
			
			for y in range(len(self.grid[x])):
				if (self.grid[x][y].accessible):
					print(".", end=" ")
				else:
					print("X", end=" ")
			print("")


	# Print a proposed path
	def printPath(self, path):
		
		if path == []:
			print("given empty path to print")

		print(" ", end="")
		for y in range(len(self.grid)):
			print(y, end=" ")
		print("")
		for x in range(len(self.grid)):
			print(x, end=" ")
			for y in range(len(self.grid[x])):

				if (self.grid[x][y] in path):
					print("P", end=" ")
				
				elif (self.grid[x][y].accessible):
					print(".", end=" ")
				else:
					print("X", end=" ")
			print("")


	# Computes the grid square from the given cartesian coordinates.
	def gridsquare(self, x, y):
		x -= self.start_x
		y -= self.start_y
		return self.grid[int(x / self.boxDim)][int(y / self.boxDim)]	


	# Computes the cartesian coordinates from the given grid square.
	def cartesian(self, gridx, gridy):
		boxDim = self.boxDim
		return (self.start_x + gridx*boxDim + boxDim/2, 
				self.start_y + gridy*boxDim + boxDim/2)
	

	# Computes the manhattan distance between two grid squares.
	def manhattan(self, nodeX, nodeY):
		return abs(nodeX.x - nodeY.x) + abs(nodeX.y - nodeY.y)

