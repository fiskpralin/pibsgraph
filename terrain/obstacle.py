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

	If you want 

	"""
	def __init__(self,pos,isSpherical, radius=500, terrain=None,color='b'):
		self.pos=pos
		self.isSpherical=isSpherical #in 2D, spherical==circular, e.g. trees (cylindrical in 3D)
		self.radius=radius #if not spherical, this is the bounding radius
		self.color=color
		self.alpha=90
		self.visible=True
		self._gridIndex=None
		if terrain:
			self.terrain=terrain
			if not (self.__class__.__name__ == 'Boulder' or isinstance(self, Process)):
				terrain.obstacles.append(self)
				terrain._insertToGrid(self)

	def getNodes(self, pos=None):
		"""this method must be overrun and implemented if not spherical."""
		if not isSpherical: raise Exception('ERROR: function getNodes is not implemented for %s. This is required.'%self.name)
		if pos and pos != self.pos: raise Exception('nodes and pos not correlated for tree.')
		return []

	def draw(self,ax):
		if self.visible:
			cir = Circle(tuple(self.pos), radius=self.radius, facecolor=self.color, alpha=self.alpha)
			ax.add_patch(cir)

