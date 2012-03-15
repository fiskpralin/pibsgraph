from SimPy.Simulation  import *
from machines.basics import UsesDriver, Operator
from functions import getCartesian, getCylindrical, getDistance, getDistanceSq
from math import pi, sin, asin, cos, sqrt
import matplotlib as mpl
import numpy as np
import copy
from terrain.pile import Pile


"""
This file is now just a copy of the heads, should be rebuilt to a bundler
"""
###################################################
# Crane and CraneHead
##################################################
class ThinningCraneHead(Process):
	"""
	common stuff for craneheads. Some of the method must be overridden, thus class should only be used as a superclass.
	"""
	def __init__(self, sim, name=None, machine=None):
		if not name: name='cranehead'
		self.m=machine
		self.s=self.m.G.simParam
		if not machine or len(self.m.heads)==0:
			self.side='left' #default
		elif len(self.m.heads)==1:
			self.side='right'
		else: raise Exception('Heads only support one or two arms on machine.')
		Process.__init__(self,name, sim)
		#self.testVar=self.m.G.paramInput['BCfd2']['testVar']
		self.timeTwigCrack=self.s['timeTwigCrack']
		self.timeCutAtHead=self.s['timeCut']
		self.treeWeight=0
		self.gripArea=0
		self.trees=[]
		self.twigCracker=True #A twigCracker is a module on the head that twigcracks the trees and cuts them into 5m long pieces
		self.currentPile=None
		
	def treeChopable(self, t):
		"""
		Determines if the cranehead can handle the tree in question.
		"""
		raise Exception('ThinningCraneHead should implement treeChopable method.')

	def harvestPos(self,t):
		"""
		function for the harvest position of a tree. Default is the trees position, but the BC head follows the road.
		"""
		return t.pos 

	def setPos(self,pos):
		"""
		Changes position for thinning crane, and records monitors associated with this.
		"""
		#record angles:
		cyl = getCylindrical(pos, origin=self.m.pos, direction=self.m.direction)
		oldCyl = getCylindrical(self.pos, origin=self.m.pos, direction=self.m.direction)
		dTh=abs(oldCyl[1]-cyl[1])
		dr=abs(oldCyl[0]-cyl[0])
		traveltime=max(dTh/self.m.velocities['crane angular'], dr/self.m.velocities['crane radial'])
		self.pos=pos
		return traveltime+self.m.times['crane const']

	def getNextTree(self, road=None):
		"""returns the tree that is closest to the machine, i.e lowest y-value in road cart.coord.sys.
			also does analysis if a tree has the desired proportions to be chopped. """
		if not road:
			if not self.road: raise Exception('getNextTree can only be called when a road is assigned')
			road=self.road
		if len(road.trees)==0: return False
		closest=100000
		t=None
		for tree in road.trees:
			d2=getDistanceSq(self.m.pos, tree.pos)
			if not tree.harvested and d2<closest:
				closest=d2
				t=tree
		if not t: print road.pos, "did not find any trees"
		return t

	def dumpTrees(self, direction=None):
		"""releases the trees at the current position. (And dumps the trees in piles)"""
		if direction is None: direction=self.road.direction
		cart=self.m.getCartesian
		t=self.getNextTree(self.road)#
		if len(self.trees)==0: return []
		if not t or self.currentPile==None:
			if self.road==self.m.roads['main']:
				self.currentPile=Pile(pos=self.m.getTreeDumpSpot(self.side),terrain=self.m.G.terrain)
				print 'Created main Pile'
			else:
				self.currentPile=Pile(pos=self.road.startPoint,terrain=self.m.G.terrain)
				print 'Created corridor Pile'
		i=0
		for tree in copy.copy(self.trees):
			tree.isSpherical=False
			tree.color='#5C3317' #brown, same as stumps
			dth=pi/30.
			direct=random.uniform(direction-dth, direction+dth)
			r=tree.radius
			l=tree.h
			a=2.0 #change to 1 to get the correct value
			c1=cart([-r, l/a], origin=self.pos, direction=direct, fromLocalCart=True)
			c2=cart([-r, 0], origin=self.pos, direction=direct, fromLocalCart=True)
			c3=cart([r, 0], origin=self.pos, direction=direct, fromLocalCart=True)
			c4=cart([r, l/a], origin=self.pos, direction=direct, fromLocalCart=True)
			tree.nodes=[c1,c2,c3,c4]
			tree.radius=sqrt(r**2+l**2)
			self.currentPile.trees.append(tree)#adds the tree to the current pile
			i=i+1
			self.trees.remove(tree)
			print 'added the',i,'th tree' 

		if len(self.trees)!=0: raise Exception('dumptrees does not remove the trees..')
		self.treeWeight=0
		self.gripArea=0
		self.currentPile.updatePile(direction)#sets pile parameters in a nice way

		if not t or getDistance(t.pos , self.m.pos)>self.m.craneMaxL: #check if more trees in this corridor or within reach in mainroad
			self.m.G.terrain.piles.append(self.currentPile)#adds the pile to the list of piles in terrain
			print '*Saved the current pile in the terrain:',len(self.currentPile.trees),'trees in pile'
			self.currentPile=None


		return self.cmnd([], time=self.timeDropTrees, auto=self.automatic['dumpTrees'])
		
	def getStartPos(self):
		if self.side=='left':
			return self.m.getCartesian([self.m.craneMinL, 2*pi/3.])
		else:
			return self.m.getCartesian([self.m.craneMinL, pi/3.])

	def roadAssigned(self):
		"""may later be used to wait for an assignment"""
		if self.road: return True
		else: return False

	def reset(self):
		self.road=None #indicates that head is passive
		self.treeWeight=0
		self.trees=[]

	def chopNext(self):
		c=[]
		CC=self.chopConst#constant for felling,zero for BC head
		t=self.getNextTree(self.road)
		if not t:
			col=self.road.color
			self.road.color='r'
			self.road.color=col
			return c
		elif t.weight+self.treeWeight>self.maxTreeWeight or t.dbh**2+self.gripArea>self.maxGripArea: #go back and dump trees if the head cannot hold any more trees
			print "Goes back to DUMP trees", self.treeWeight, self.gripArea
			if self.road == self.m.roads['main']: time=self.setPos(self.m.getTreeDumpSpot(self.side))
			else: time=self.setPos(self.road.startPoint)
			self.cmnd(c, time, auto=self.automatic['moveArmIn'])
			c.extend(self.dumpTrees()) #dumps them down.
		
		elif not getDistance(t.pos , self.m.pos)>self.m.craneMaxL:
			time=self.setPos(self.harvestPos(t))
			self.cmnd(c, time, auto=self.automatic['moveArmOut'])
			#determine choptime
			cross_sec_area=t.dbh**2*pi
			choptime=CC+cross_sec_area/self.velFell
			self.cmnd(c, choptime, auto=self.automatic['chop'])
			t.pos=[5000, 5000] #far away, for visual reasons.
			t.h-=0.5 #harvester is at least half a meter above ground
			t.harvested=True
			self.road.trees.remove(t)
			self.road.harvestTrees-=1
			self.trees.append(t)
			self.treeWeight+=t.weight
			self.gripArea+=t.dbh**2
			self.m.trees.append(t)
			self.m.treeMoni.observe(len(self.m.trees), self.sim.now())
		return c

	def twigCrack(self):
		if self.twigCracker:
			#trees should be be twigcracked and cut!
			#not yet implemented
			#HERE: change volume of pile and its length acc to model by Ola and Dan
			#How get to work on the pile in the list terrain.piles?
			time=self.timeTwigCrack+self.timeCutAtHead
			print 'Trees have been twigcracked, it took', time, 'seconds'
			return self.cmnd([], time, auto=self.automatic['twigCrack'])
		else:
			print 'Trees not twig cracked'
			return []

	def getCraneMountPoint(self):
		"""returns the point where the crane meets the head"""
		cart=self.m.getCartesian
		cyl=self.m.getCylindrical(self.pos)
		if not self.road or self.direction==pi/2.: #head is at "base" or with default direction
			return(cart([cyl[0]-self.length/2., cyl[1]]))
		return cart([0, -self.length/2], origin=self.pos, direction=self.direction, fromLocalCart=True)
