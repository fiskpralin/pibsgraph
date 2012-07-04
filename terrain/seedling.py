#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path

from tree import *
class Seedling(Tree):
	def __init__(self, pos, radius=0.01 ,terrain=None,dbh=None, height=None, weight=None, vol=None, plantMinDist=1.5):
		Tree.__init__(self, pos, radius=radius, terrain=terrain ,dbh=dbh, height=height, weight=weight, vol=vol)
		self.name = "TreeSeedling"
		if plantMinDist!=None:
			self.plantMinDist=plantMinDist
		else:
			try:
				self.plantMinDist=terrain.G.simParam['dibbleDist']#terrain.G.plantMinDist
			except: #either terrain=None or terrain does not have G or G.simParam does not have 'dibbleDist' key
				self.plantMinDist=1.5 #default
			print self.plantMinDist, terrain.G.simParam['dibbleDist']#terrain.G.plantMinDist
	def draw(self, ax):
		cir = Circle(tuple(self.pos), radius=self.plantMinDist, facecolor=self.color, alpha=95)
		ax.add_patch(cir)
		cir2=Circle(tuple(self.pos), radius=self.radius, facecolor=self.color, alpha=95)
		ax.add_patch(cir2)
