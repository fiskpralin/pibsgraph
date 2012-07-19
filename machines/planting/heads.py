from SimPy.Simulation  import *
import machines
from machines.basics import Machine, UsesDriver, Operator
import terrain
from terrain.obstacle import Obstacle
from terrain.tree import Tree
from terrain.hole import Hole
from terrain.seedling import Seedling
from terrain.boulder import Boulder
from collision import *
from functions import *
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pylab
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Circle, Rectangle 
import copy
import time
from math import *

###################################################
## Plant head
####################################################
class PlantHead(Process, UsesDriver, Obstacle):
	"""This class represents a planthead. It does simple things and is completely governed by its plantingdevice. Actions occur here when they do not necessarily happen simultaneously in both plantheads."""
	def __init__(self, name, sim, PD,width=None):
		Process.__init__(self, name, sim)
		UsesDriver.__init__(self, PD.driver)
   		self.p=PD #planting device
		self.length=2*self.p.G.simParam['moundRadius']
		self.depth=self.length*0.5 #radius of hole
		if self.p.G.simParam['wMB'] != None:
			self.width=self.p.G.simParam['wMB']
		else:
			if width==None:
				raise Exception('needs a width for the plantHead. Either give it or set simParam[wMB]')
			self.width=width
		self.timeConsumption={'mounding':0, 'planting':0, 'halting':0, 'inverting':0}
		self.reMoundSignal=SimEvent(name='planthead remounded', sim=self.sim)
		self.reset()
		
	def run(self):
		raise Exception('subclasses of PlantHead must implement run')
	
	def debugPrint(self, string):
		print "%f %s: %s"%(self.sim.now(), self.name, string)
		
	def cmndWithDriver(self, commands, time):
		"""overrides superclass method, adds priority"""
		if self.usesDriver: #don't need to reserve driver..
			commands.extend([(hold, self, time)])
			#if a is None: print "uses driver", self.name, [].append((hold, time)), commands.append((hold, time)),hold, time
			return commands
		elif self.p.usesDriver or len(self.p.plantHeads)>1 and len([h for h in self.otherHeads if h.usesDriver])>0: #set a high priority.
			prio = 2
		else:
			prio=1
		commands.append((request, self, self.driver, prio))
		if prio==1: #append switch focus time
			switchTime=self.p.m.times['switchFocus']
			if self.driver.isIdle():
				switchTime-=self.driver.idleTime()
				if switchTime<0: switchTime=0
			commands.append((hold, self, switchTime))#switchtime
			self.p.m.timeConsumption['switchFocus']+=switchTime
		commands.append((hold, self, time))
		self.usesDriver=True #this means that a reservation from the driver has been sent, not that he currently has the attention here.
		return commands
	def reset(self):
		"""called before a new cycle begins."""
		self.moundSuccess=False
		self.mounded=False
		self.extended=False
		self.abort=False
		self.moundObst=[]
		self.strikedImmobile=False
		self.moundSumA=0.
		self.biggestBlock=0.
		self.done=False
		self.remounded=False
		self.cause=None
	def getPos(self, plant=False):
		"""returns the position of the plantHead"""
		p=self.p
		if isinstance(self, Bracke):
			return p.pos #does this really work? don't know..
		if not plant: [p1,p2]=p.getPHCoord(p.posCyl, 'cylindrical')
		else: [p1,p2]=p.getPlantingCoord(p.posCyl, 'cylindrical')
		if self.leftRight=='right':
			return p2
		elif self.leftRight=='left':
			return p1
		else:
			raise Exception("Error, leftright is undefined for plantHead.")
	def getNodes(self, pos=None):
		if pos is None: pos=self.getPos()
		return self.p.getPhNodes(self, pos)
	def scramble(self):
		"""scramble the moundobst. This function is invoked after mound is complete and dibble is about to begin. Simply create a heap at the location of the planthead. Dimension is length,width adn depth specified above but with depth as positive. Heap is a half circle extended as a cylinder in width-dimension."""
		pos=self.getPos()
		p=self.p
		l=self.length #i.e length of "tracks" 
		w=self.width
		h=self.depth
		uniform=random.uniform
		olist=[]
		direction=p.m.direction-pi/2.+p.posCyl[1]
		for o in self.moundObst:
			placed=False
			while not placed:
				x=uniform(-l/2., l/2.)
				y=uniform(-w/2., w/2.)
				z=uniform(0, sqrt(y**2+(l/2.)**2))
				placed=True
				for b in olist:
					if sqrt(pow(x-b.pos[0],2)+pow(y-b.pos[1],2)+pow(z-b.z,2))<o.radius+b.radius:
						placed=False
						break
				o.pos=p.m.getCartesian([x, y],origin=pos, direction=direction, local=False, fromLocalCart=True)
				o.z=z
				olist.append(o)
		self.moundObst=olist
	def rootCollide(self, root):
		#assumes that position is the one determined by plantingDevice
		pos=self.getPos()
		cyl=self.p.m.getCylindrical(pos)
		nodes=root.getNodes()
		r=rect(pos, self.p.getPhNodes(self, pos=None, rootA=True))
		if collide(r, root):
			if pointInPolygon(pos, nodes):
				print "root covers the centre of the hole"
				col=root.z+root.diameter/2.>-self.depth*0.75 #compare depths
				return col
			else:
				P=closestPolygonPoint(pos, nodes)
				localP=getCartesian(P, origin=self.pos, direction=cyl[1], local=True)
				col=(root.z+root.diameter/2.)**2+localP[1]**2<(0.75*self.depth)**2
				return col
		else: return False
#####################################
#Multihead
#####################################
class MultiHead(PlantHead):
	"""
	A head that is part of a device with several heads.

	Number starts from 0 and the left part of the device. So in a 4-head device, the head on the right is number 3.
	"""
	def __init__(self, name, sim, PD, number,width=0.4):
		PlantHead.__init__(self,name, sim, PD, width)
		self.number=number
		Obstacle.__init__(self, [0,0], isSpherical=False, radius=sqrt((self.width/2.)**2+(self.length/2.)**2), terrain=PD.G.terrain)
		self.color=['y','r','g','b'][self.number]
	def run(self):
		"""
		Observe that much of the works done by the head is simulated in the
		planting device's "plant" method. This is because this work is done simulataneously
		for all the heads and thus is simulated in the device.

		Everything done here should be separate things that may be finished at different
		heads belonging to the same device.
		"""
		p=self.p
		debugPrint=self.debugPrint
		self.otherHeads=copy.copy(p.plantHeads)
		self.otherHeads.remove(self)
		assert self in p.plantHeads
		digTime=self.p.m.getDigTime(p.pos) #defined to be the same for all the heads given a position
		while True:
			self.sleeping=True #indicates that we are in the below hold
			yield hold, self, 100000000 #until interupted
			self.sleeping=False
			if self.interrupted():
				cause=self.cause
				self.pos=self.getPos()
				if cause=='Plant' or cause=='plant':
					#When this is invoked, the pDev is assumed to be waiting for this event.
					auto=p.m.automatic
					t=p.m.times
					p.m.stopControl()
					if self.moundSumA < 0.08: #sumA smaller than 8dm2
						lim=sqrt((0.10**2)/pi)#model has rectangular stones. 10cm side
						if self.biggestBlock < lim:
							debugPrint("plants, no of stones:%f sumA: %f biggestBl:%f"%(len(self.moundObst),self.moundSumA,self.biggestBlock))
						elif self.strikedImmobile: #boulder did not occupy more than 50% of z-axis, or root was correctly aligned.
							assert not p.G.simParam['noRemound'] #we should not be here in that case
							#remound, this is guaranteed to work..
							if not self.remounded:
								debugPrint("striked immobile, remounds")
								self.sim.stats['remound attempts']+=1
								self.remounded=True
								for otherHead in self.otherHeads:
									otherHead.remounded=True

									otherHead.cause='reMound'
									yield hold, self, 0.0000001 #to let the other head wake up
									self.interrupt(self.otherHead) #cannot do anything with other head while this head works..
									debugPrint("interrupts other head.")
									yield hold, self, 0.0000001 #time for other head to release driver
								assert len(self.reMoundSignal.waits)+len([h for h in self.otherHeads if h.sleeping])<=len(self.otherHeads)
								for c in self.cmnd([], digTime+t['heapTime'],auto['mound']): yield c
								self.timeConsumption['mounding']+=digTime+t['heapTime']
								self.reMoundSignal.signal()
						else: #biggestblock between limits, check if dibble succeeds.
							#scramble boulders.
							self.scramble()
							#pos=copy.copy(self.pos)
							pos=copy.copy(self.pos) #we don't move head, just dibble
							boulList=[b for b in self.moundObst if isinstance(b, Boulder) and b.volume>0.001]
							self.sim.stats['number of dibble disr stones in mound']+=len(boulList)
							self.sim.stats['dibble distr stone vol cum']+=sum([b.volume for b in boulList])
							for posDiff in [[0,0], [0,0.05], [0,-0.10]]:
								pos[0]+=posDiff[0]
								pos[1]+=posDiff[1]
								done=True
								for o in boulList:
									dist2=pow(o.pos[0]-pos[0],2)+pow(o.pos[1]-pos[1],2)
									depth=self.depth-p.m.dibbleDepth #positive
									if dist2<o.radius**2 and o.z>=depth and o.volume>0.001:
										print "failed dibble"
										done=False
										time=t['dibbleDownTime']+t['dibbleUpTime']
										for c in self.cmnd([], time, auto['plant']):
											yield c
											for cm in self.checkIfInterupted(): yield cm
										self.sim.stats['plant attempts']+=1
										self.timeConsumption['planting']+=time
										break
								if done: 
									debugPrint("dibble succeded, plants")
									break
							if not done: #remound, always works..
								if self.G.simParam['noRemound']:
									self.abort=True
									break
								debugPrint("remounds")
								self.sim.stats['remound attempts']+=1
								self.remounded=True
								for otherHead in self.otherHeads:
									otherHead.remounded=True
									otherHead.cause='reMound'
									self.interrupt(otherHead) #cannot do anything with other head while this head works..
									yield hold, self, 0.0000001 #time for other head to release driver
									self.debugPrint("interrupts other head.")
								assert len(self.reMoundSignal.waits)+len([h for h in self.otherHeads if h.sleeping])<=len(self.otherHeads) #less if two heads reMounds at the same time
								for c in self.cmnd([], digTime+t['heapTime'],auto['mound']): yield c
								self.timeConsumption['mounding']+=digTime+t['heapTime']
								self.reMoundSignal.signal()
								debugPrint("plants after remound/heap.")
					else: #moundSumA was to much..
						for c in self.cmnd([], t['haltTime'], auto['haltMound']):
							yield c
							for cm in self.checkIfInterupted(): yield cm
						self.timeConsumption['halting']+=t['haltTime']
						self.abort=True #should later check if it was successfull.
						debugPrint("striked >4dm2: %f, aborts"%self.moundSumA)
					if not self.abort: #plant
						self.debugPrint("plants...")
						self.sim.stats['plant attempts']+=1
						tree=Seedling(pos=self.getPos(plant=True),radius=0.025, terrain=p.G.terrain, plantMinDist=self.p.m.plantMinDist)
						plantTime=t['dibbleDownTime']+t['relSeedlingTime']+t['dibbleUpTime']
						for c in self.cmnd([], plantTime, auto['plant']):
							yield c
							for cm in self.checkIfInterupted(): yield cm
						self.timeConsumption['planting']+=plantTime
						debugPrint("done with yielding")
						p.m.treesPlanted.append(tree)
						p.m.treeMoni.observe(len(p.m.treesPlanted), self.sim.now())
						p.m.stopControl()
					debugPrint("signals!")
					p.plantSignals+=1
					for c in self.releaseDriver(): yield c
					yield hold, self, 0.000000001 #to give some time for entities waiting before going "passive"
					debugPrint("goes into long hold")
					self.reset()
				elif cause=='mound' or cause=='reMound':
					#mounds again..
					debugPrint("interrupted from sleep because of remound.. sleeps again...")
					self.reset() #planthead was idle, and can remain idle.
				else:
					raise Exception("Planthead could not recognice interruptcause: %s"%cause)
	def getPos(self, plant=False):
		"""
		returns the position of the plantHead
		"""
		p=self.p
		if not plant:
			posList=p.getPHCoord(p.posCyl, 'cylindrical')
		else:
			posList=p.getPlantingCoord(p.posCyl, 'cylindrical')
		return posList[self.number]
	def checkIfInterupted(self):
		cmd=[]
		if self.interrupted():
			if self.cause=='reMound':
				t=self.interruptLeft
				auto=True
				if self.usesDriver: auto=False #the task we interupted was not automated
				cmd.extend(self.releaseDriver())
				otherHead=self.interruptCause
				cmd.extend([(waitevent, self, otherHead.reMoundSignal)])
				cmd=self.cmnd(cmd,t, auto)
			elif self.cause is not 'plant':
				raise Exception('only interupt possible is remound, except plant. But plant interuptions should not be possible when checkifinterrupted is called.')
		return cmd
		
#####################################
#Mplanter
#####################################
class Mplanter(MultiHead):
	"""
	only difference in the heads is so far the width.

	Big differences in the device, but that is on another level.
	"""	
	def __init__(self, name, sim, PD, number):
		defaultWidth=0.45
		MultiHead.__init__(self,name, sim, PD, number, defaultWidth)

###########################
# Bracke
###########################	
class Bracke(MultiHead):
	def __init__(self, name, sim, PD):
		defaultWidth=0.4
		MultiHead.__init__(self,name, sim, PD, number=0,width=defaultWidth)
		








