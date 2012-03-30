#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path

from obstacle import *
class Tree(Obstacle):
	def __init__(self, pos, radius=None,terrain=None ,dbh=None, height=None, weight=None, logWeight=None, vol=None, specie=None, gvl_75=None, dstump=None):
		Obstacle.__init__(self, pos, isSpherical=True, radius=radius, terrain=terrain, color='g')
		self.z=0
		self.logWeight=logWeight
		if not dbh:
			if not radius:
				raise('ERROR: Tree needs either radius or dbh, otherwise colission detection goes bananas.')
			dbh=radius*2
		self.dbh=dbh
		if not radius: radius=dbh/2.
		self.radius=radius
		if not height: height=self.dbh*20 #no connection to reality..
		if not vol:
			vol=(dbh*0.5)**2*pi*height*0.333 #cone
		self.vol=vol
		if not weight:
			weight=530*self.vol #this is for oregon pine, but works for now.. 
		self.weight=weight
		self.h=height
		self.name = "Tree"
		self.harvested=False
		self.specie=specie
		self.gvl_75=gvl_75
		self.dstump=dstump
		if terrain: terrain.trees.append(self)
		self.nodes=None
		self.marked=False #marked for harvesting
		#we omit the rest for now..
	def getNodes(self, pos=None):
		if self.isSpherical and not self.nodes:
			return False
		else:
			if not self.nodes: raise Exception('non-spherical trees must have nodes')
			return self.nodes
	def draw(self, ax):
		if not self.isSpherical and self.nodes: #plot as polygon.. often harvested.
			ax.add_patch(mpl.patches.Polygon(np.array(self.nodes), closed=True, facecolor=self.color, alpha=85))
		else:
			ax.add_patch(Circle(tuple(self.pos), radius=self.radius, facecolor=self.color))
