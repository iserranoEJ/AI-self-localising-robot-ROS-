from math import floor

class PriorityQueue:
	
	
	# Constructor. Takes an initial node as input.
	def __init__(self, initNode):

		self.heap = [initNode]
		self.nodePositions = {initNode: 0}

	
	# Determines whether this priority queue 
	# contains the given node.
	def contains(self, node):
		return node in self.nodePositions


	# Enqueues the given node in the heap
	def enqueue(self, node):

		self.heap.append(node)
		self.updatePosition(node, self.lastNodeIndex())
		self.swim(self.lastNodeIndex())


	# Returns and removes the highest priority element in this queue
	# Restores heap order afterwards.
	def dequeue(self):

		minElement = self.heap[0]
		self.swap(0, self.lastNodeIndex())
		self.heap.pop(self.lastNodeIndex())
		del self.nodePositions[minElement]
		self.sink(0)
		return minElement
		

	# Restores heap order if the key of the given
	# node has been decreased. Assumes the rest of 
	# the heap is in heap order.
	def decreaseKey(self, node):

		nodeIndex = self.nodePositions[node]
		self.swim(nodeIndex)
		
	
	# Restores heap order from the subheap rooted
	# at the given node.
	def sink(self, currentNode):

		leftChild = self.leftChild(currentNode)
		rightChild = self.rightChild(currentNode)
		minNode = currentNode

		if(self.inbounds(leftChild) and self.heap[leftChild] < self.heap[minNode]):
			minNode = leftChild
		
		if(self.inbounds(rightChild) and self.heap[rightChild] < self.heap[minNode]):
			minNode = rightChild

		if(minNode != currentNode):
			self.swap(currentNode, minNode)
			self.sink(minNode)
	

	# Restores heap order following the addition of a node
	# to the tail of the heap.
	def swim(self, currentNode):

		parent = self.parent(currentNode)
		while(self.inbounds(parent) and self.heap[currentNode] < self.heap[parent]):
			self.swap(currentNode, parent)
			currentNode = parent
			parent = self.parent(currentNode)


	# Helper functions for maintaining heap order.

	
	# Updates the known position of the given node
	# within the heap
	def updatePosition(self, node, index):
		self.nodePositions[node] = index

	# Swap the contents of two nodes and updates
	# their known positions.
	def swap(self, nodeXIndex, nodeYIndex):

		self.nodePositions[self.heap[nodeXIndex]] = nodeYIndex
		self.nodePositions[self.heap[nodeYIndex]] = nodeXIndex
		
		temp = self.heap[nodeXIndex]
		self.heap[nodeXIndex] = self.heap[nodeYIndex]
		self.heap[nodeYIndex] = temp

	
	# Get the index of the left child of this node
	def leftChild(self, nodeIndex):
		return 2 * nodeIndex + 1

	# Get the index of the right child of this node
	def rightChild(self, nodeIndex):
		return 2 * nodeIndex + 2

	# Get the index of the parent of this node
	def parent(self, nodeIndex):
		return (nodeIndex - 1) // 2

	# Returns the index of the last node (most recently added)
	# in the heap.
	def lastNodeIndex(self):
		return len(self.heap) - 1

	# Determine whether the supplied index is within 
	# bounds of the heap.
	def inbounds(self, nodeIndex):
		return nodeIndex >= 0 and nodeIndex < len(self.heap)
	
