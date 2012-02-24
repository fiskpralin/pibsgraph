from SimPy.Monitor import *  
from SimPy.Simulation  import *
from SimPy.Monitor import *

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
	def __init__(self, name, sim, PD):
		Process.__init__(self, name, sim)
		UsesDriver.__init__(self, PD.driver)
   		self.p=PD #planting device
		self.timeConsumption={'mounding':0, 'planting':0, 'halting':0}
		"""
		Uptdated:
		None..
		"""
		self.reMoundSignal=SimEvent(name='planthead remounded', sim=self.sim)
		self.reset()
	def run(self):
		raise Exception('subclasses of PlantHead must implement run')
	def debugPrint(self, string):
		print "%f %s: %s"%(self.sim.now(), self.name, string)
	def cmndWithDriver(self, commands, time):
		"""overrides superclass method, adds priority"""
		print "command with driver"
		if self.usesDriver: #don't need to reserve driver..
			commands.extend([(hold, self, time)])
			#if a is None: print "uses driver", self.name, [].append((hold, time)), commands.append((hold, time)),hold, time
			return commands
		elif self.p.usesDriver or len(self.p.plantHeads)>1 and self.otherHead.usesDriver: #set a high priority.
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
		print "does not use driver"
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
				self.debugPrint('collided with root: %s'%(str(col)))
				return col
			else:
				P=closestPolygonPoint(pos, nodes)
				localP=getCartesian(P, origin=self.pos, direction=cyl[1], local=True)
				col=(root.z+root.diameter/2.)**2+localP[1]**2<(0.75*self.depth)**2
				self.debugPrint('collided with root: %s'%(str(col)))
				return col
		else: return False

		
#####################################
#Mplanter
#####################################
class Mplanter(PlantHead):
	def __init__(self, name, sim, PD, leftRight):
		PlantHead.__init__(self,name, sim, PD)
		self.leftRight=leftRight #'right' or 'left'
		self.length=0.4 #i.e length of "tracks" 
		if self.p.G.PMbladeWidth:
			self.width=self.p.G.PMbladeWidth
		else:
			self.width=0.45 #default
		self.depth=0.2
		Obstacle.__init__(self, [0,0], isSpherical=False, radius=sqrt((self.width/2.)**2+(self.length/2.)**2), terrain=PD.G.terrain)
	def run(self):
		p=self.p
		debugPrint=self.debugPrint
		if self.leftRight=='right':
			self.otherHead=p.plantHeads[0]
			self.color='r'
		else:	#left
			self.otherHead=p.plantHeads[1]
			self.color='y'
		while True:
			yield hold, self, 100000000 #until interupted
			if self.interrupted():
				cause=self.cause
				self.pos=self.getPos()
				if cause=='Plant' or cause=='plant':
					"""When this is invoked, the pDev is assumed to be waiting for this event."""
					auto=p.m.automatic
					t=p.m.times
					if self.moundSumA < 0.08: #sumA smaller than 8dm2
						lim=sqrt((0.10**2)/pi)#model has rectangular stones. 10cm side
						if self.biggestBlock < lim:
							debugPrint("plants, no of stones:%f sumA: %f biggestBl:%f"%(len(self.moundObst),self.moundSumA,self.biggestBlock))
						elif self.strikedImmobile: #boulder did not occupy more than 50% of z-axis, or root was correctly aligned.
							#remound, this is guaranteed to work..
							if not self.remounded:
								debugPrint("striked immobile, remounds")
								self.sim.stats['remound attempts']+=1
								self.otherHead.remounded=True
								self.remounded=True
								yield hold, self, 0.0000001 #to let the other head wake up
								self.otherHead.cause='reMound'
								self.interrupt(self.otherHead) #cannot do anything with other head while this head works..
								debugPrint("interrupts other head.")
								yield hold, self, 0.0000001 #time for other head to release driver
								for c in self.cmnd([], t['moundAndHeapTime'],auto['mound']): yield c
								self.timeConsumption['mounding']+=t['moundAndHeapTime']
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
								self.sim.stats['plant attempts']+=1
								for o in boulList:
									dist2=pow(o.pos[0]-pos[0],2)+pow(o.pos[1]-pos[1],2)
									depth=self.depth-p.m.dibbleDepth #positive
									if dist2<o.radius**2 and o.z>=depth and o.volume>0.001:
										print "failed dibble"
										done=False
										for c in self.cmnd([], t['dibbleDownTime'], auto['plant']):
											yield c
											for cm in self.checkIfInterupted(): yield cm
										self.timeConsumption['planting']+=t['dibbleDownTime']
										break
								if done: 
									debugPrint("dibble succeded, plants")
									break
							if not done: #remound, always works..
								debugPrint("remounds")
								self.sim.stats['remound attempts']+=1
								self.otherHead.cause='reMound'
								self.interrupt(self.otherHead) #cannot do anything with other head while this head works..
								self.debugPrint("interrupts other head.")
								for c in self.cmnd([], t['moundAndHeapTime'],auto['mound']): yield c
								self.timeConsumption['mounding']+=t['moundAndHeapTime']
								
								self.reMoundSignal.signal()
								#if self.otherHead.passive(): yield reactivate, self.otherHead
								debugPrint("plants after remound/heap.")
					else: #moundSumA was to much..
						for c in self.cmnd([], t['haltTime'], auto['haltMound']):
							yield c
							for cm in self.checkIfInterupted(): yield cm
						for hT in [self, self.otherHead]: hT.timeConsumption['halting']+=t['haltTime']
						
						self.abort=True #should later check if it was successfull.
						debugPrint("striked >4dm2: %f, aborts"%self.moundSumA)
					if not self.abort: #plant
						self.debugPrint("plants...")
						self.sim.stats['plant attempts']+=1
						tree=Seedling(pos=self.getPos(plant=True),radius=0.025, terrain=p.G.terrain)
						plantTime=t['dibbleDownTime']+t['relSeedlingTime']
						for c in self.cmnd([], plantTime, auto['plant']):
							yield c
							for cm in self.checkIfInterupted(): yield cm
						self.timeConsumption['planting']+=t['dibbleDownTime']
						debugPrint("done with yielding")
						p.m.treesPlanted.append(tree)
						p.m.treeMoni.observe(len(p.m.treesPlanted), self.sim.now())
					debugPrint("signals!")
					p.plantSignals+=1
					for c in self.releaseDriver(): yield c
					yield hold, self, 0.000000001 #to give some time for entities waiting before going "passive"
					debugPrint("goes into long hold")
					self.reset()
				elif cause=='mound' or cause=='reMound':
					"""mounds again.."""
					debugPrint("interrupted from sleep because of remound.. sleeps again...")
					self.reset() #planthead was idle, and can remain idle.
					pass
				else:
					raise Exception("Planthead could not recognice interruptcause: %s"%cause)
	def checkIfInterupted(self):
		cmd=[]
		if self.interrupted():
			if self.cause=='reMound':
				t=self.interruptLeft
				cmd.extend(self.releaseDriver()+[(waitevent, self, self.otherHead.reMoundSignal), (hold,self,t)])
			elif self.cause is not 'plant':
				raise Exception('only interupt possible is remound, except plant. But plant interuptions should not be possible when checkifinterrupted is called.')
		return cmd


###########################
# Bracke
###########################	
class Bracke(PlantHead):
	def __init__(self, name, sim, PD):
		PlantHead.__init__(self,name, sim, PD)
		self.length=0.4 #i.e length of "tracks" 
		if self.p.G.PMbladeWidth:
			self.width=self.p.G.PMbladeWidth
		else:
			self.width=0.4 #default
		self.depth=0.2
		Obstacle.__init__(self, [0,0], isSpherical=False, radius=sqrt((self.width/2.)**2+(self.length/2.)**2), terrain=PD.G.terrain)
	def run(self):
		p=self.p
		while True:
			yield hold, self, 100000000
			if self.interrupted():
				cause=self.cause
				self.pos=self.getPos()
				if cause=='Plant' or cause=='plant':
					"""When this is invoked, the pDev is assumed to be waiting for this event."""
					auto=p.m.automatic
					t=p.m.times
					if self.moundSumA < 0.08: #sumA smaller than 8dm2
						lim=sqrt((0.10**2)/pi)#model has rectangular stones. 10cm side
						if self.biggestBlock < lim:
							self.debugPrint("plants, no of stones:%f sumA: %f biggestBl:%f"%(len(self.moundObst),self.moundSumA,self.biggestBlock))
						elif self.strikedImmobile: #boulder did not occupy more than 50% of z-axis, or root was correctly aligned.
							#remound, this is guaranteed to work..
							if not self.remounded:
								self.sim.stats['remound attempts']+=1
								self.debugPrint("striked immobile, remounds")
								self.remounded=True
								yield hold, self, 0.0000001 #to let the other head wake up
								for c in self.cmnd([], t['moundAndHeapTime'],auto['mound']): yield c
								self.timeConsumption['mounding']+=t['moundAndHeapTime']
								self.reMoundSignal.signal()
						else: #biggestblock between limits, check if dibble succeeds.
							#scramble boulders.
							self.scramble()
							#pos=copy.copy(self.pos)
							pos=copy.copy(self.pos)
							boulList=[b for b in self.moundObst if isinstance(b, Boulder) and b.volume>0.001]
							self.sim.stats['number of dibble disr stones in mound']+=len(boulList)
							self.sim.stats['dibble distr stone vol cum']+=sum([b.volume for b in boulList])
							for posDiff in [[0,0], [0,0.05], [0,-0.10]]:
								pos[0]+=posDiff[0]
								pos[1]+=posDiff[1]
								done=True
								self.sim.stats['plant attempts']+=1
								for o in boulList:
									dist2=pow(o.pos[0]-pos[0],2)+pow(o.pos[1]-pos[1],2)
									depth=self.depth-p.m.dibbleDepth #positive
									if dist2<o.radius**2 and o.z>=depth:
										done=False
										for c in self.cmnd([], t['dibbleDownTime'], auto['plant']):
											yield c
											for cm in self.checkIfInterupted(): yield cm
										self.timeConsumption['planting']+=t['dibbleDownTime']	
										break
								if done: 
									self.debugPrint("dibble succeded, plants")
									break
							if not done: #remound, always works..
								self.debugPrint("remounds")
								self.sim.stats['remound attempts']+=1
								for c in self.cmnd([], t['moundAndHeapTime'],auto['mound']): yield c
								self.timeConsumption['mounding']+=t['moundAndHeapTime']
								self.reMoundSignal.signal()
								#if self.otherHead.passive(): yield reactivate, self.otherHead
								self.debugPrint("plants after remound/heap.")
					else: #moundSumA was to much..
						for c in self.cmnd([], t['haltTime'],auto['haltMound']):
							yield c
							for cm in self.checkIfInterupted(): yield cm
						self.timeConsumption['halting']+=t['haltTime']
						self.abort=True #should later check if it was successfull.
						self.debugPrint("striked >4dm2: %f, aborts"%self.moundSumA)
					if not self.abort: #plant
						self.debugPrint("plants...")
						self.sim.stats['plant attempts']+=1
						tree=Seedling(pos=self.getPos(plant=True),radius=0.025, terrain=p.G.terrain)
						plantTime=t['dibbleDownTime']+t['relSeedlingTime']
						for c in self.cmnd([], plantTime, auto['plant']):
							yield c
							for cm in self.checkIfInterupted(): yield cm
						self.timeConsumption['planting']+=t['dibbleDownTime']
						self.debugPrint("done with yielding")
						p.m.treesPlanted.append(tree)
						p.m.treeMoni.observe(len(p.m.treesPlanted), self.sim.now())
					self.debugPrint("signals!")
					p.plantSignals+=1
					for c in self.releaseDriver(): yield c
					yield hold, self, 0.000000001 #to give some time for entities waiting before going "passive"
					self.debugPrint("goes into long hold")
					self.reset()
				elif cause=='mound' or cause=='reMound':
					raise Exception('Bracke planter should not be intterrupted because of remounding')
				else:
					raise Exception("Planthead could not recognice interruptcause: %s"%str(cause))
	def checkIfInterupted(self):
		cmd=[]
		if self.interrupted():
			if self.cause=='reMound':
				t=self.interruptLeft
				cmd.extend(self.releaseDriver()+[(waitevent, self, self.otherHead.reMoundSignal), (hold,self,t)])
			elif self.cause is not 'plant':
				raise Exception('only interupt possible is remound, except plant. But plant interuptions should not be possible when checkifinterrupted is called.')
		return cmd













