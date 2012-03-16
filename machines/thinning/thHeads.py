from SimPy.Simulation  import *

from machines.basics import UsesDriver, Operator
from functions import getCartesian, getCylindrical, getDistance, getDistanceSq

from math import pi, sin, asin, cos, sqrt
import matplotlib as mpl
import numpy as np
import copy

from terrain.pile import Pile


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
		self.bundler=False
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
		if not self.bundler:
			"""releases the trees at the current position. (And dumps the trees in piles)"""
			if direction is None: direction=self.road.direction
			cart=self.m.getCartesian
			t=self.getNextTree(self.road)#
			c=[]
			if len(self.trees)==0: return []
			if not t or self.currentPile==None:
				if self.road==self.m.roads['main']:
					self.currentPile=Pile(pos=self.pos,terrain=self.m.G.terrain)
					print '*Created main Pile'
				else:
					self.currentPile=Pile(pos=self.pos,terrain=self.m.G.terrain)
					print '*Created corridor Pile'
			i=0
			for tree in copy.copy(self.trees):
				tree.isSpherical=False
				tree.nodes=[[-0.1,1],[-0.1,0],[0.1,0],[0.1,-1]]
				tree.pos=[5000,5000]
				""" #Here the trees arechanged, but we dont need it since all is saved in the piles. I HOPE ;)! 
				tree.color='#5C3317' #brown, same as stumps
				dth=pi/30.
				direct=random.uniform(direction-dth, direction+dth)
				r=tree.radius
				l=tree.h
				a=1.0 #change to 1 to get the correct value
				c1=cart([-r, l/a], origin=self.pos, direction=direct, fromLocalCart=True)
				c2=cart([-r, 0], origin=self.pos, direction=direct, fromLocalCart=True)
				c3=cart([r, 0], origin=self.pos, direction=direct, fromLocalCart=True)
				c4=cart([r, l/a], origin=self.pos, direction=direct, fromLocalCart=True)
				tree.nodes=[c1,c2,c3,c4]
				tree.radius=sqrt(r**2+l**2)
				"""
				self.currentPile.trees.append(tree)#adds the tree to the current pile
				i=i+1
				self.trees.remove(tree)
				print 'added the',i,'th tree' 

			if len(self.trees)!=0: raise Exception('dumptrees does not remove the trees..')
			self.treeWeight=0
			self.gripArea=0
			self.currentPile.updatePile(direction)#sets pile parameters in a nice way
			c.extend(self.twigCrack())
			if not t or getDistance(t.pos , self.m.pos)>self.m.craneMaxL: #check if more trees in this corridor or within reach in mainroad
				self.m.G.terrain.piles.append(self.currentPile)#adds the pile to the list of piles in terrain
				print '*Saved the current pile in the terrain:',len(self.currentPile.trees),'trees in pile'
				self.currentPile=None
			self.cmnd(c, time=self.timeDropTrees, auto=self.automatic['dumpTrees'])
			return c

		else:
			"""releases the trees at the current position. (And dumps the trees in piles)"""
			if direction is None: direction=self.road.direction
			cart=self.m.getCartesian
			t=self.getNextTree(self.road)#
			c=[]
			if len(self.trees)==0: return []
			if self.currentPile==None:
				self.currentPile=Pile(pos=self.m.pos,terrain=self.m.G.terrain)#HERE ADD ITS CORRECT POSITION
			i=0
			for tree in copy.copy(self.trees):
				tree.isSpherical=False
				tree.nodes=[[-0.1,1],[-0.1,0],[0.1,0],[0.1,-1]]
				tree.pos=[5000,5000]
				self.currentPile.trees.append(tree)#adds the tree to the current pile
				i=i+1
				self.trees.remove(tree)
				print 'added the',i,'th tree' 

			if len(self.trees)!=0: raise Exception('dumptrees does not remove the trees..')
			self.treeWeight=0
			self.gripArea=0
			self.currentPile.updatePile(direction)#sets pile parameters in a nice way
			c.extend(self.twigCrack())
			for t in self.currentPile.trees:
				bundler.currentBundle.trees.append(t)
				print 'moved the trees from the cP of head to cB of bundler', len(self.m.Bundler.currentBundle.trees), 'are now in that bundle'
				bundler.currentBundle.updatePile(pi/2)#can i do this? probably wrong direction
				self.currentPile=None
			self.cmnd(c, time=self.timeDropTrees, auto=self.automatic['dumpTrees'])
			return c
	
		
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
		"""
		Changes the volume of the pile and its length according
		to models by Ola and Dan (see doc/models.txt). Also the properties of the pile is
		updated.
		"""
		if self.twigCracker and self.currentPile:
			self.currentPile.updatePile(self.road.direction)
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

			
class BCHead(ThinningCraneHead, UsesDriver):
	"""This is the cranehead of Rickard and julia"""
	def __init__(self, sim, driver, machine):
		UsesDriver.__init__(self,driver)
		ThinningCraneHead.__init__(self, sim, name="BCHead", machine=machine)
		self.road=None #the correct road will be stored here
		self.pos=self.getStartPos()
		self.m.heads[self.side]=self
		self.color=self.m.color
		self.reset()
		self.direction=pi/2.
		self.maxTreeWeight=self.s['maxWeightCD']#350 #kg
		self.width=self.s['headWidthCD']#1.
		self.corridorWidth=self.width
		self.corrPerSide=self.s['noCorridorsPerSideCD']
		self.length=self.s['headWidthCD']+0.5
		self.timeDropTrees=self.s['timeDropTreesCD']
		self.chopConst=self.s['constFellTreeCD']
		self.velFell=self.s['velocityFellTreeCD']
		self.maxGripArea=self.s['maxGripAreaCD']
		self.automatic={'moveArmIn': self.s['moveArmInCD'],
						'moveArmOut': self.s['moveArmOutCD'],
						'dumpTrees': self.s['dropTreesCD'],
						'chop': self.s['fellTreesCD'],
						'twigCrack': self.s['twigCrackD']}
		
		
	def run(self):
		while True:
			yield waituntil, self, self.roadAssigned
			if self.road==self.m.roads['main']: 
				#clear the mainroad.
				roadList=[self.road]
				self.road.startPoint=self.pos
			else:
				#road is a thinning corridor.
				if len(self.m.heads)==2:
					roadList=self.m.roads[self.m.pos[1]][self.side]
					if roadList[0] is not self.road: raise Exception('road system does not work as expected.%s, %s, %s'%(self.side, self.road, roadList[0]))
				else:
					a=self.m.roads[self.m.pos[1]].values() #two list with 3 roads in each
					roadList=a[0]+a[1]
			for road in roadList:
				self.road=road
				mainRoad=True
				sPoint=road.startPoint
				if self.road != self.m.roads['main']:
					mainRoad=False
					time=self.setPos(sPoint)
					for c in self.cmnd([], time, auto=self.automatic['moveArmOut']): yield c
				while True:
					cmd=self.chopNext()
					if len(cmd)==0: break
					for c in cmd: yield c
				#trees have been gathered. return.
				time=self.setPos(sPoint)
				if mainRoad: time+=self.setPos(self.m.getTreeDumpSpot(self.side))
				for c in self.cmnd([], time, auto=self.automatic['moveArmIn']): yield c	
				for c in self.dumpTrees(): yield c #dumps them down.
			for c in self.releaseDriver(): yield c
			print "done at site", self.pos
			self.reset()
			
	def harvestPos(self,t):
		"""harvest position of tree, along road. Overrides default, pos of tree"""
		if not self.road or self.road==self.m.roads['main']: return super(BCHead, self).harvestPos(t)
		s=self.road.startPoint
		[r,th]=self.m.getCylindrical(t.pos, origin=s, direction=self.road.direction)
		yloc=sin(th)*r #0<th<pi
		return self.m.getCartesian([0, yloc], origin=s, direction=self.road.direction, fromLocalCart=True)

	def treeChopable(self, t):
		"""determines if a tree is chopable."""
		if t.dbh>0.10 or t.specie=='spruce' or t.h>10:
			return False
		else: return True

	def setPos(self, pos):
		#determine direction.. from current pos to next pos. If going backwards, use the default..
		if pos==self.road.startPoint or pos==self.m.getTreeDumpSpot(self.side): #going back..
			[r, self.direction]=self.m.getCylindrical(pos)
		else: [r,self.direction]=getCylindrical(pos,origin=self.pos, direction=pi/2.)
		return super(BCHead, self).setPos(pos)

   	def draw(self, ax):
		cart=self.m.getCartesian
		#crane:
		wC=0.2
		[r,th]=self.m.getCylindrical(self.getCraneMountPoint())
		direct=self.m.direction-pi/2.+th
		h1=cart([wC/2., 0],direction=direct, fromLocalCart=True)
		h2=cart([wC/2., r],direction=direct, fromLocalCart=True)
		h3=cart([-wC/2., r], direction=direct, fromLocalCart=True)
		h4=cart([-wC/2., 0], direction=direct, fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([h1,h2,h3,h4]), closed=True, facecolor='k'))
		#head
		inOutRatio=0.7
		c=self.m.getCartesian
		W=self.width
		w=inOutRatio*W
		L=self.length
		if not self.road:
			direction=self.m.direction-pi/2.+self.m.getCylindrical(self.pos)[1]
		else:
			direction=self.direction
		c1=c([W/2., L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		c2=c([-W/2., L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		c3=c([-w/2., -L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		c4=c([w/2., -L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor=self.color))


class ConventionalHeadAcc(ThinningCraneHead, UsesDriver):
	"""
	This is a conventional cranehead for thinning with a possibility to accumulate some trees.
	"""
	def __init__(self, sim, driver, machine):
		UsesDriver.__init__(self,driver)
		ThinningCraneHead.__init__(self, sim, name="ConvHeadAcc", machine=machine)
		self.road=None #the currect road will be stored here
		self.pos=self.getStartPos()
		self.m.heads[self.side]=self
		self.color=self.m.color
		self.trees=[]
		self.reset()
		self.direction=pi/2.
		self.length=self.s['headWidthEF']#0.5 OBSERVE THAT THIS IS HEAD IS SET TO BE SQUARE
		self.width=self.s['headWidthEF']#0.5
		self.corridorWidth=self.s['corridorWidthEF']#2
		self.corrPerSide=self.s['noCorridorsPerSideEF']#3
		self.maxTreeWeight=self.s['maxWeightEF']#350 #set it to same as BC...
		self.maxGripArea=self.s['maxGripAreaEF']#0.03#0.28 #[m2]is reasonable: comes from max grip radius of 0.3m
		self.timeDropTrees=self.s['timeDropTreesEF']
		self.chopConst=self.s['constFellTreeEF']
		self.velFell=self.s['velocityFellTreeEF']
		self.maxGripArea=self.s['maxGripAreaEF']
		self.automatic={'moveArmIn': self.s['moveArmInEF'],
						'moveArmOut': self.s['moveArmOutEF'],
						'dumpTrees': self.s['dropTreesEF'],
						'chop': self.s['fellTreesEF'],
						'twigCrack': self.s['twigCrackF']}

	def run(self):
		while True:
			yield waituntil, self, self.roadAssigned
			if self.road==self.m.roads['main']: 
				#clear the mainroad.
				roadList=[self.road]
				sPoint=self.pos
			else:
				#road is a thinning corridor.
				if len(self.m.heads)==2:
					roadList=self.m.roads[self.m.pos[1]][self.side]
					if roadList[0] is not self.road: raise Exception('road system does not work as expected.%s, %s, %s'%(self.side, self.road, roadList[0]))
				else:
					a=self.m.roads[self.m.pos[1]].values() #two list with 3 roads in each
					roadList=a[0]+a[1]
			for road in roadList:
				self.road=road
				mainRoad=True
				print "starts harvesting"
				if self.road != self.m.roads['main']:
					mainRoad=False
					sPoint=road.startPoint
					time=self.setPos(sPoint)
					for c in self.cmnd([], time, auto=self.automatic['moveArmOut']): yield c
				while True:
					cmd=self.chopNext()
					if len(cmd)==0: break
					for c in cmd: yield c
				time=self.setPos(sPoint) # trees have been gathered. return to machine after each maxAcc
				if mainRoad: time+=self.setPos(self.m.getTreeDumpSpot(self.side))
				for c in self.cmnd([], time, auto=self.automatic['moveArmIn']): yield c #
				for c in self.dumpTrees(): yield c #dumps the trees
			for c in self.releaseDriver(): yield c
			print "done at site", self.pos
			self.reset()

	def treeChopable(self, t):
		"""determines if a tree is chopable."""
		if t.dbh>0.10 or t.h>10:
			return False
		else: return True
		
   	def draw(self, ax):
		cart=self.m.getCartesian
		#crane:
		wC=0.2
		[r,th]=self.m.getCylindrical(self.getCraneMountPoint())
		direct=self.m.direction-pi/2.+th
		h1=cart([wC/2., 0],direction=direct, fromLocalCart=True)
		h2=cart([wC/2., r],direction=direct, fromLocalCart=True)
		h3=cart([-wC/2., r], direction=direct, fromLocalCart=True)
		h4=cart([-wC/2., 0], direction=direct, fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([h1,h2,h3,h4]), closed=True, facecolor='k'))
		#head
		c=self.m.getCartesian
		W=self.width
		L=self.length
		direction=self.m.direction-pi/2.+self.m.getCylindrical(self.pos)[1]
		c1=c([W/2., L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		c2=c([-W/2., L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		c3=c([-W/2., -L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		c4=c([W/2., -L/2.],origin=self.pos, direction=direction, fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor=self.color))
