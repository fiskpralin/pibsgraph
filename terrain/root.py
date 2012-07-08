#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path

from obstacle import *

class Root(Obstacle):
	cap=0.6 #roots bigger than this are split
	def __init__(self, pos, Bradius, diameter, z, nodes, direction=None, terrain=None, stump=None, visible=False):
		Obstacle.__init__(self, pos, isSpherical=False, radius=Bradius, terrain=terrain)
		self.z=z
		if z>0:
			raise Exception('Root cannot have height >0')
		self.diameter=diameter
		self.nodes=nodes
		self.direction=direction
		self.visible=visible #visible roots are part of the stump
		self.stump=stump
		if self.stump:
			self.color=stump.color
		if terrain:
			self.terrain=terrain
			self.name="root%d"%len(self.terrain.roots)
			self.terrain.roots.append(self)
		else:
			self.name="root"
			
	def draw(self,ax):
		alpha=(not self.visible)*150+1
		p=mpl.patches.Polygon(np.array(self.nodes), closed=True, facecolor=self.color, alpha=alpha)
		ax.add_patch(p)
		#ax.annotate("%0.2f"%self.z, xy=tuple(self.pos)) #draw the depth
		
	def getNodes(self, pos=None):
		if pos and self.pos != pos: raise Exception('Root pos is constant.')
		return self.nodes
