#!/usr/bin/env python
from math import *
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
from obstacle import *


class Pile(Obstacle):
	
	def __init__(self, pos, biomass=None, diameter=None, radius=None, terrain=None , length=None, weight=None, logWeight=None, vol=None):
		Obstacle.__init__(self, pos, isSpherical=False, radius=radius, terrain=terrain, color='brown')# '#5C3317'after debug
		self.pos=pos
		self.radius=radius #This Radius thingy is for collision detection, don't bother about it.
		self.vol=vol
		self.trees=[]
		self.biomass=biomass
		self.length=length
		self.diameter=diameter
		self.name = "Pile"
		self.nodes=None

	def updatePile(self, direction=None):
		if direction is None: raise('EXCEPTION: updatePile needs the direction of the Road')
		
		if not self.length:
			self.length = max([t.h for t in self.trees])
		else: self.length=5 #cut!
		
		self.maxDiam = max([t.dbh for t in self.trees])
		if not self.diameter:
			self.diameter = 0.1748 + 0.0345 * self.maxDiam*100 # This model is given by Ola and Dan 2012.03.10
		else: self.diameter = self.diameter*(1-((self.diameter*61.364-0.3318)/100)) #twigcrack!
		
		if not self.biomass:
			self.biomass = sum([t.weight for t in self.trees])#initial weight no losses
		else: self.biomass*(1-((self.diameter*17.237-3.9036)/100))#with losses from twigcracking
		
		self.radius = sqrt(self.length**2+(self.diameter/2)**2)

		#here the nodes of the pile are set
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
	def __init__(self, pos):
		Pile.__init__(self, pos, biomass=None, diameter=None, radius=None, terrain=None , length=None, weight=None, logWeight=None, vol=None)

	def updatePile(self, direction=None):
		#This should be updated such that it works for the bundler
		if direction is None: raise('EXCEPTION: updatePile needs the direction of the Road')
		
		if not self.length:
			self.length = max([t.h for t in self.trees])
		else: self.length=5 #cut!
		
		self.maxDiam = max([t.dbh for t in self.trees])
		
		if not self.diameter:
			self.diameter = 0.1748 + 0.0345 * self.maxDiam*100 # This model is given by Ola and Dan 2012.03.10
		else: self.diameter = self.diameter*(1-((self.diameter*61.364-0.3318)/100)) #twigcrack!
		
		if not self.biomass:
			self.biomass = sum([t.weight for t in self.trees])#initial weight no losses
		else: self.biomass*(1-((self.diameter*17.237-3.9036)/100))#with losses from twigcracking
		
		self.radius = sqrt(self.length**2+(self.diameter/2)**2)

		#here the nodes of the pile are set
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
			ax.add_patch(mpl.patches.Polygon(np.array(self.nodes), closed=True, facecolor='grey', alpha=85))
		else:
			raise('EXCEPTION: The pile must have nodes.')
