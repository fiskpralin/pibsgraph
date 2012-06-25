#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
from obstacle import *

class SurfaceBoulder(Obstacle):
	
	def __init__(self, pos, radius, z, terrain=None):
		Obstacle.__init__(self, pos, isSpherical=True, radius=radius, terrain=terrain, color='grey')
		#for now, boulders are spherical with their position in the middle.
		self.z = z
		self.radius = radius
		self.volume = pow(radius,3)*pi*4.0/3.0
		self.area = (radius**2)*pi #area projected to the surface
		self.name = "surfaceboulder"
		self.visible = True

	def draw(self, ax):
		if not self.isSpherical: raise Exception('surfaceboulder should be spherical')
		else:
			ax.add_patch(Circle(tuple(self.pos), radius=self.radius, facecolor=self.color))
