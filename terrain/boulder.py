#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path

from obstacle import *

class Boulder(Obstacle):
	def __init__(self, pos, radius, z):
		Obstacle.__init__(self, pos, isSpherical=True, radius=radius)
		#for now, boulders are spherical with their position in the middle.
		self.z=z
		self.radius=radius
		self.volume=pow(radius,3)*pi*4.0/3.0
		self.area=(radius**2)*pi #area projected to the surface
		if radius>z:
			visible=True
		self.name="boulder"
