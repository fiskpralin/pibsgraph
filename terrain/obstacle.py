#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path

class Obstacle():
	"""
	Superclass for all obstacles.
	"""
	def __init__(self,pos,isSpherical, radius=None, terrain=None,color='b'):
		self.pos=pos
		self.isSpherical=isSpherical #in 2D, spherical==circular, e.g. trees (cylindrical in 3D)
		self.radius=radius #if not spherical, this is the bounding radius
		self.color=color
		self.alpha=90
		self.visible=True
		self._gridIndex=None
		if terrain:
			self.terrain=terrain
			if not (self.__class__.__name__ == 'Boulder' or isinstance(self, Process)): #ugly if statement.. but needed for boulder performance..
				terrain.addObstacle(self)

	def getNodes(self, pos=None):
		"""this method must be overrun and implemented if not spherical."""
		if not isSpherical: raise Exception('ERROR: function getNodes is not implemented for %s. This is required.'%self.name)
		if pos and pos != self.pos: raise Exception('nodes and pos not correlated for obstacle.')
		return []

	def remove(self):
		"""
		removes obstacle from the associated lists. See terrain.remove method for more info
		"""
		if self.terrain:
			self.terrain.remove(self)

	def draw(self,ax):
		if self.visible:
			cir = Circle(tuple(self.pos), radius=self.radius, facecolor=self.color, alpha=self.alpha)
			ax.add_patch(cir)

