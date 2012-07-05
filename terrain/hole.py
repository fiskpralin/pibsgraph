#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
from obstacle import *


class Hole(Obstacle):
	"""
	A hole in the ground, right now used to mark that ground has been digged at specific spot
	"""
	def __init__(self,pos,radius=None,terrain=None,z=0, nodes=None, isSpherical=True):
		Obstacle.__init__(self, pos, isSpherical=isSpherical, radius=radius, terrain=terrain)
		self.name="hole"
		self.z=z
		self.nodes=nodes
		self.color='b'
		self.alpha=90
		if radius==None:
			if isSpherical:
				raise Exception('Hole without nodes needs a radius')
			radius=1e10
			for node in nodes:
				d=getDistance(node, pos)
				if d<radius:
					radius=d
		self.radius=radius
		if terrain: terrain.holes.append(self)
		
	def getNodes(self, pos):
		return self.nodes
	
	def draw(self, ax):
		codes=[Path.MOVETO]
		for i in range(4):#one more than nodes
			codes.append(Path.LINETO)
		codes.append(Path.CLOSEPOLY) #two more than nodes
		path = Path(self.nodes+[self.nodes[0], [1,1]], codes)
		patch = mpl.patches.PathPatch(path, facecolor=self.color, alpha=self.alpha)
		ax.add_patch(patch)
