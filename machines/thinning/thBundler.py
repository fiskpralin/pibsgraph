from SimPy.Simulation  import *
from machines.basics import UsesDriver, Operator
from functions import getCartesian, getCylindrical, getDistance, getDistanceSq
from math import pi, sin, asin, cos, sqrt
import matplotlib as mpl
import numpy as np
import copy
from terrain.pile import Pile, Bundle


"""
This file is now just a copy of the heads, should be rebuilt to a bundler
"""
###################################################
# Bundler
##################################################
class Bundler(Process,UsesDriver):
	"""
	This is the Bundler Class to be positioned in front of the machine. Will be run from machine.py I guess
	with some sort of wait until
	"""
	def __init__(self, sim, driver, machine, name="Bundler"):
		UsesDriver.__init__(self,driver)
		Process.__init__(self, name, sim)
		
		self.m=machine
		self.s=self.m.G.simParam
		self.timeBundle=self.s['timeBundle']
		self.bundleWeight=0
		self.xSection=0
		self.maxXSection=self.s['maxXSectionJ']
		self.xSectionThresh=self.s['xSectionThreshJ']
		self.currentBundle=None

	def run(self):
		"""
		PEM of the bundler class
		"""
		"""
		yield waituntil self.xSection>=self.xSectionThresh #or until self.xSection + new trees would be > self.maxXSection
		for c 
		#change the "drop position" for the heads
		#store trees in bundler until it reaches threshold
		#make bundle and dump the trees on the side of the main road
		"""
		time = 1
		yield hold, self, time
		
#FROM the cranehead still
	def dumpTrees(self, direction=None):
		"""releases the trees at the current position. (And dumps the trees in piles)"""
		if direction is None: direction=pi/2
		cart=self.m.getCartesian
		self.treeWeight=0
		self.gripArea=0
		self.currentPile.updatePile(direction)#sets pile parameters in a nice way
		self.m.G.terrain.piles.append(self.currentBundle)#adds the pile to the list of piles in terrain
		print '*Saved the current bundle in the terrain:',len(self.currentBundle.trees),'trees in that Bundle'
		self.currentPile=None
		return self.cmnd([], time=self.timeDropTrees, auto=self.automatic['dumpTrees']) #Should this time really be there?
		
	def roadAssigned(self):
		"""may later be used to wait for an assignment"""
		if self.road: return True
		else: return False

	def reset(self):
		self.road=None #indicates that head is passive
		self.bundleWeight=0
		self.xSection=0
		self.currentBundle=None
		

	def getCraneMountPoint(self):
		"""returns the point where the crane meets the head"""
		cart=self.m.getCartesian
		cyl=self.m.getCylindrical(self.pos)
		if not self.road or self.direction==pi/2.: #head is at "base" or with default direction
			return(cart([cyl[0]-self.length/2., cyl[1]]))
		return cart([0, -self.length/2], origin=self.pos, direction=self.direction, fromLocalCart=True)

	def draw(self):
		pass 
