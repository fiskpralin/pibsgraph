#!/usr/bin/env python
from math import *
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
from obstacle import *


class Pile(Obstacle):
	"""
	A class for describing piles of trees. These can be trees gathered separately or accumulated at head. If they
	are left at the same positin they build a pile. This pile can be twigcracked, cut, etc. Also it is superclass
	for Bundle and inherits from obstacle. So it is a visible non-spehriacl obstacle with nodes.
	"""
	def __init__(self, pos, biomass=None, diameter=None, radius=None, terrain=None , length=None, weight=None, logWeight=None, vol=None):
		Obstacle.__init__(self, pos, isSpherical=False, radius=radius, terrain=terrain, color='#5C3317')
		self.pos = pos
		self.radius = radius #This Radius thingy is for collision detection, don't bother about it.
		self.vol = vol
		self.trees = []
		self.biomass = biomass
		self.length = length
		self.diameter = diameter
		self.name = "Pile"
		self.nodes = None
		self.ficDiameter = 0
		self.xSection=None
		self.craneCycles=0#This is for simulation statistics

	def updatePile(self, direction=None):
		if direction is None: raise('EXCEPTION: updatePile needs the direction of the Road')
		if not self.length:
			self.length = max([t.h for t in self.trees])
		self.maxDiam = max([t.dbh for t in self.trees])
		
		self.diameter = 0.1748 + 0.0345 * self.maxDiam*100 # This model is given by Ola and Dan 2012.03.10
		self.biomass = sum([t.weight for t in self.trees])#initial weight no losses
		
		self.radius = sqrt(self.length**2+(self.diameter/2)**2)
		self.setNodes(direction)

	def twigCrackPile(self,direction):
		"""
		This is the twigcracking of the Pile. The model for reduction of biomass and volume (diameter) is
		given by Dan Bergstrom at SLU.
		"""
		self.length = 5 #cut!
		self.maxDiam = max([t.dbh for t in self.trees])
		self.biomass = self.biomass*(1-((self.diameter*17.237-3.9036)/100))#with losses from twigcracking
		self.diameter = self.diameter*(1-((self.diameter*61.364-0.3318)/100)) #twigcrack!
		self.radius = sqrt(self.length**2+(self.diameter/2)**2)
		self.setNodes(direction)

	def setNodes(self,direction=None):
		#here the nodes of the pile are set
		if direction is None: raise('EXCEPTION: updatePile needs the direction of the Road')
		c1=getCartesian([-self.diameter/2,self.length], origin=self.pos, direction=direction, fromLocalCart=True)
		c2=getCartesian([-self.diameter/2, 0], origin=self.pos, direction=direction, fromLocalCart=True)
		c3=getCartesian([self.diameter/2, 0], origin=self.pos, direction=direction, fromLocalCart=True)
		c4=getCartesian([self.diameter/2,self.length], origin=self.pos, direction=direction, fromLocalCart=True)
		self.nodes=[c1,c2,c3,c4]
		
	def getNodes(self, pos=None):
		if not self.nodes:
			return False
		else:
			return self.nodes
		
	def draw(self, ax):
		if self.nodes: #plot as polygon.
			ax.add_patch(mpl.patches.Polygon(np.array(self.nodes), closed=True, facecolor=self.color, alpha=85))
		else:
			raise('EXCEPTION: The pile must have nodes.')

class Bundle(Pile):
	"""
	This is a class which inherits from Pile. It is used when the thinningmachine is equipped with a bundler.
	That means all trees are dumped at the bundler which produces a bundle, a certain type of pile which is
	compressed in a different way than with the twigcracker.
	"""
	def __init__(self, pos, biomass=None, diameter=None, radius=None, terrain=None, length=None, weight=None, logweight=None, vol=None):
		Pile.__init__(self, pos, biomass=None, diameter=None, radius=None, terrain=None , length=None, weight=None, logWeight=None, vol=None)
		self.xSection=0

	def updatePile(self, direction=None):
		"""
		This should be updated such that it works for the bundler. Does only set som parameters
		for the current bundle, does not execute the bundling, that is done in method in thBundler.py
		Overrides updatePile in Pile.
		"""
		if direction is None: direction= pi/2
		self.length = max([t.h for t in self.trees])
		self.maxDiam = max([t.dbh for t in self.trees])
  		self.biomass = sum([t.weight for t in self.trees])#initial weight no losses
		self.xSection = sum([self.getXSection(tree=t) for t in self.trees])# This models what we compare with thresh in bundler
		self.diameter = sqrt(self.xSection*4/pi)
		self.ficDiameter = 0.1748 + 0.0345 * self.maxDiam*100 # This model is given by Ola and Dan 2012.03.10, only to be able to calculate the lost biomass at twig crack
		self.radius = sqrt(self.length**2+(self.diameter/2)**2)
		self.setNodes(direction)

	def getXSection(self, tree=None):
		"""
		Adds the cross section of the stump and at five meters for each tree. Uses a linear function for the
		thickness of the stem. The model from Dan has been changed to resemble an ordinary linear function
		instead  but it is actually still exactly the same.
		"""
		if tree is None: raise('EXCEPTION: getXSection must get a tree')
		stumpXS=tree.dstump**2
		if tree.h<5: atFiveXS=0
		elif tree.gvl_75==0 or tree.gvl_75==1.3:
			atFiveXS=(tree.dbh+(tree.dbh-tree.dstump)/1.3*3.7)**2
		else:
			atFiveXS=(tree.dbh+(0.075-tree.dbh)/(tree.gvl_75-1.3)*3.7)**2
		return stumpXS+atFiveXS	

	def twigCrackBundle(self,direction):
		self.maxDiam = max([t.dbh for t in self.trees])
		self.biomass = self.biomass*(1-((self.ficDiameter*17.237-3.9036)/100))#with losses from twigcracking
		self.setNodes(direction)
		
	def draw(self, ax):
		if self.nodes: #plot as polygon.
			ax.add_patch(mpl.patches.Polygon(np.array(self.nodes), closed=True, facecolor='#5C3317', alpha=85))
		else:
			raise('EXCEPTION: The bundle must have nodes.')
