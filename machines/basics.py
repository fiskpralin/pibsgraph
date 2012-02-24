from SimPy.Monitor import *  
from SimPy.Simulation  import *
from SimPy.Monitor import *

from math import *
import terrain
from terrain.obstacle import Obstacle
from terrain.tree import Tree
from terrain.hole import Hole
from terrain.seedling import Seedling
from collision import *
from functions import *

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pylab
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle 
import copy
import time
"""This module stores all the machines."""

###################################################
## Machine
####################################################
class Machine(Process, Obstacle):
	"""
	This is the superclass for all machines. """
	def __init__(self,name, sim,G,driver=None, pos=[0,0], mass=None, direction=pi/2., shape=None):
		"""
		initializes the machine.
		If the shape parameter is given it has information about the nodes in relation to the center (pos) of the machine.
		Convenient way of handling the colission detection, but not really needed.
		"""
		Process.__init__(self, name, sim)
		self.pos=pos
		self.G=G
		self.mass=21000
		self.direction=direction #the direction of the machine.
		self.color='y'
		self.velocities={'machine':1} #implement in subclass
		self.times={} # implement in subclass
		self.visited=[]
		self.sim.machines.append(self) #adds to global list of machines..
		self.moveEvent=None #could be signaled if needed, nbut omit as default to save time
		if isinstance(self, UsesDriver): #if machine inherits from driver, default is no.
			if not driver: raise Exception('Error: Machine %s uses driver, but no driver was given to MAchine.__init__.'%self.name)
			UsesDriver.__init__(self,driver)
		#the shape of the machine:
		if not shape: #default.. 
			x=1.5
			y=1.5
			self.shape=[[-x, y], [-x, -y], [x, -y], [x, y]]
		else:
			self.shape=shape
		#find the bounding radius
		self.radius=0
		for node in self.shape:
			c=self.getCartesian(node, fromLocalCart=True)
			d=getDistance(c, self.pos)
			if d>self.radius:
				self.radius=d
		Obstacle.__init__(self,pos, isSpherical=False, radius=self.radius)
	def setPos(self,pos):
		"""
		Changes position for machine, and records monitors associated with this.
		"""
		distance=getDistance(self.pos, pos)
		if self.moveEvent: self.moveEvent.signal() #tell the world that a movement is about to occur.
		traveltime=distance/self.velocities['machine']
		self.pos=pos
		return traveltime
	def getCartesian(self,posCyl,origin=None, direction=None, local=False, fromLocalCart=False):
		if not origin: origin=self.pos
		if direction==None: direction=self.direction
		a=getCartesian(posCyl,origin=origin, direction=direction, local=local, fromLocalCart=fromLocalCart)
		return a
	def getCylindrical(self,pos, origin=None, direction=None):
		"""
		nice way of calling the global function, omitts a lot of arguments and makes direction simpler
		"""
		if not origin: origin=self.pos
		if direction==None: direction=self.direction
		cylPos=getCylindrical(pos, origin=origin, direction=direction)
		return cylPos
	def getNodes(self, pos):
		"""
		This method is suposed to be overridden, needed if machine should be used in colission detection.
		"""
		raise Exception('ERROR: function getNodes is not implemented for %s. This is required.'%self.name)
	def draw(self, ax):
		"""
		draws machine, starting from the nodes.
		"""
		codes=[]
		verts=self.getNodes()
		codes.append(Path.MOVETO)
		for corner in verts[1:]:
			codes.append(Path.LINETO)
		verts.append((5,5)) #ignored
		codes.append(Path.CLOSEPOLY)
		path = Path(verts, codes)
		patch = patches.PathPatch(path, facecolor=self.color, lw=2)
		ax.add_patch(patch)
		
###################################################
## Operator
####################################################
class Operator(Resource,Process):
	"""a machine operator. Both a resource and a process."""
	def __init__(self,sim, restTime=1.5, activeTime=15, delay=1):
		Resource.__init__(self,capacity=1, name='driver',qType=PriorityQ, monitored=True, monitorType=Monitor, sim=sim)
		Process.__init__(self, name=self.name, sim=self.sim)
		self.activeTime=activeTime #s
		self.restTime=restTime
		self.resting=False
		self.delay=delay
		self.lastRest=0
	def work(self):
		"""The PEM of driver. Manages rests. """
		prio=1000 #before anything else
		while True:
			yield hold, self, self.delay
			if self.timeToRest():
				print self.sim.now(), " operator takes a rest"
				reqTime=self.sim.now() #requestTime
				yield request, self, self, prio
				self.resting=True
				yield hold, self, self.restTime
				self.resting=False
				self.lastRest=self.sim.now()
				self.fixMonitorAfterRest(reqTime)
				yield release, self, self
				yield hold, self, self.activeTime-self.delay #saves some computations
	def timeToRest(self):
		"""determines if it is time to take a rest, based on the activeTime parameter and the monitor for recent use of driver."""
		if self.n != 0:	return False #operator is not busy
		time=self.sim.now()
		if time-self.lastRest<=self.activeTime: return False #beginning of sim
		mon=self.actMon #second element indicates if driver is used.
		last= mon[len(mon)-1]
		if last[1]==0: raise Exception('timeToRest does not work as planned.')
		#We are looking for a 0 followed by at least restTime time units until next 1.
		for a in reversed([-1,0]+mon): #step backwards. [-1,0] removes a bug caused by len(monList)<=2
			if time-last[0]>self.activeTime: return True #time to rest.. 
			if a[1]==0: #found a rest-spot.. but how big is it?
				if last[0]-a[0]>self.restTime: return False #rest was big enough.
			last=a
		raise Exception('timeToRest reached the end, should by design not be possible')
	def fixMonitorAfterRest(self,reqTime):
		"""to show that driver rests, and does not work, also shows the waitingQueue corectly"""
		mon=self.actMon
		mon[len(mon)-1][1]=0
		#self.actMon=self.actMon[0:len(self.actMon)-2]
		#fix the waitingmonitor
		time=self.sim.now()
		for a in reversed(self.waitMon):
			if a[0]<reqTime: break
			if a[0]<(time-self.restTime): a[1]-=1
	def isIdle(self):
		if self.n != 0: return True
		else: return False
	def idleTime(self):
		"""returns the time that the driver has been idle for."""
		t=self.sim.now()
		last= self.actMon[len(self.actMon)-1] #last "time stamp"
		if last[1]==1: raise Exception('idleTime can only be called when idle')
		return t-last[0]
		
####################################################
## UsesDriver
####################################################
class UsesDriver():
	"""
	objects that uses the driver can inherit from here, and use the methods.
	Used for managing the driver.
	"""
	def __init__(self, driver):
		self.usesDriver=False #set to True if have a request in queue for the driver. We don't necessarily have
		#access to the driver at the moment.
		self.driver=driver
	def cmnd(self, commands, time,auto):
		"""returns a command, simplifies driver checks."""
		if auto:
			return self.cmndAuto(commands, time)
		else:
			return self.cmndWithDriver(commands, time)
	def cmndWithDriver(self, commands, time):
		"""a method to set up the yield command, for use of the driver for a specific time."""
		if self.usesDriver: #don't need to reserve driver..
			commands.extend([(hold, self, time)])
		else:
			commands.extend([(request, self, self.driver), (hold, self, time)])
			self.usesDriver=True
		return commands
	def cmndAuto(self, commands, time):
		"""a method for the yield command, with automatic."""
		commands.extend(self.releaseDriver())
		commands.extend([(hold, self, time)])
		return commands	
	def releaseDriver(self):
		if self.usesDriver:
			self.usesDriver=False
			return [(release, self, self.driver)]
		else:
			return []
	def driverBusy(self):
		if self.driver.n == 0:
			return True
		else:
			return False
		
######################
# MovesCont()
#######################
class MovesCont():
	"""
	for continous movements, inherit from this class.
	NOT TESTED ENOUGH. Still have restrictions on movements, be aware.
	"""
	def __init__(self):
		self.contSetPos(self.pos, 0)
	def contSetPos(self,pos, time, oldPos=None):
		if not oldPos: oldPos=self.pos
		self._pIni=oldPos
		self._pFin=pos
		t=self.sim.now()
		self._tIni=t
		self._tFin=t+time
	def getPos(self):
		print "getPos was called", self._tIni, self._tFin
		t=self.sim.now()
		if t-self._tFin>=0: return self.pos
		pIni=np.array(self._pIni)
		pFin=np.array(self._pFin)
		d=pFin-pIni #vector between the two points
		return list(self._pIni+(t-self._tIni)/(self._tFin-self._tIni)*d)

