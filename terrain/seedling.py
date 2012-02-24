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
		if terrain and terrain.G: self.plantMinDist=terrain.G.plantMinDist
		else: self.plantMinDist=plantMinDist
	def draw(self, ax):
		cir = Circle(tuple(self.pos), radius=self.plantMinDist, facecolor=self.color, alpha=95)
		ax.add_patch(cir)
		cir2=Circle(tuple(self.pos), radius=self.radius, facecolor=self.color, alpha=95)
		ax.add_patch(cir2)
