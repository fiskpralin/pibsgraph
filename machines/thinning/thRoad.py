from road import Road
import collision as col

class ThinningRoad(Road):
	"""
	a thinning road. Connected to machine in a bad way right now..
	"""
	def __init__(self, pos, nodes, G,direction=None, machine=None, main=False, radius=None):
		Road.__init__(self, pos, nodes, G, direction,radius=radius)
		self.m=machine
		self.trees=[]
		self.harvestTrees=0 #used to take care of the overlapping problem..
		tL=G.terrain.GetTrees(self.pos, self.radius+1) #+1 to be sure...
		for tree in tL:
			if col.pointInPolygon(tree.pos, self.getNodes()): #tree inside road,
				#check if tree already belongs to a road:
				if not self._checkIfTaken(tree):
					self.harvestTrees+=1
				self.trees.append(tree)
		self.startPoint=None #the startpoint of a corridor is where it intersects the mainRoad.This is where the crane is supposed to begin.
		self.main=main

	def _checkIfTaken(self,tree):
		"""
		checks all the roads... very ineffective
		"""
		if not self.m: return False #cannot iterate roads.. 
		for road in self.m.roadList:
			for t in road.trees:
				if t is tree:
					return True
		return False
	
	def add(self):
		"""
		adds road into roadlist.. pretty ugly routine..
		"""
		if not self.m:
			raise Exception('ThinningRoad is not connected to a machine. cannot add road.')
		if self.main: #this is the main road
			self.m.mainRoadTrees.extend(self.trees)
		else:
			for t in self.trees:
				if self.m.corridorTrees.count(t)==0: self.m.corridorTrees.append(t)


		self.m.roadList.append(self)
	def draw(self, ax):
		super(ThinningRoad, self).draw(ax)
