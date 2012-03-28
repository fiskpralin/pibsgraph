from SimPy.Simulation  import *
from machines.basics import UsesDriver, Operator
from functions import getCartesian, getCylindrical, getDistance, getDistanceSq
from math import pi, sin, asin, cos, sqrt
import matplotlib as mpl
import numpy as np
import copy
from terrain.pile import Pile, Bundle


###################################################
# Bundler
##################################################
class Bundler(Process,UsesDriver):
	"""
	This is the Bundler Class to be positioned in front of the machine. Will be run from machine.py
	"""
	def __init__(self, sim, driver, machine, name="Bundler"):
		UsesDriver.__init__(self,driver)
		Process.__init__(self, name, sim)
		self.color='blue'
		self.m=machine
		self.s=self.m.G.simParam
		self.pos=np.array(self.m.pos)+np.array([0,self.s['dropPosJ']])
		self.timeBundle=self.s['timeBundle']
		self.maxXSection=self.s['maxXSectionJ']
		self.xSectionThresh=0.1#self.s['xSectionThreshJ']
		self.currentBundle=None
		self.forceBundler=False #Is set to true when bundler is filled or new pile from head does not fit
		
	def run(self):
		"""
		PEM of the bundler class.
		This method checks for a condition to run which is when it filled above threshold
		or the incoming pile does not fit. It makes a bundle with some set properties
		end dump it beside the road. 
		"""
		while True:
			yield waituntil, self, self.bundlerFilled
			print 'The bundler runs and makes a bundle of the pile'
			for c in self.startTheBundler(): yield c
			self.bundleIt()
			self.dumpBundle()
			self.resetBundle()
			for c in self.cmnd([],self.s['timeBundle']-self.s['timeStartBundler'],auto=self.s['restOfBundling']): yield c
			for c in self.releaseDriver(): yield c
			print 'end of bundlerrun'
		
	def dumpBundle(self, direction=None):
		"""
		Releases the bundle at the current position. (And dumps the bundle in terrain)
		"""
		if direction is None: direction=pi/2

		#here the nodes of the bundle are set when the bundle is put in the terrain
		cB=self.currentBundle
		cB.pos=np.array(self.pos)+np.array([-2.5,0])#puts it beside the main road doesn't work though
		c1=getCartesian([-cB.diameter/2,cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		c2=getCartesian([-cB.diameter/2, -cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		c3=getCartesian([cB.diameter/2, -cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		c4=getCartesian([cB.diameter/2,cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		cB.nodes=[c1,c2,c3,c4]
		
		self.m.G.terrain.piles.append(cB)#adds the pile to the list of piles in terrain
		self.m.G.terrain.addObstacle(cB)
		print '*SAVED the current bundle with',len(self.currentBundle.trees),'trees in the terrain at pos:',cB.pos

	def getBPos(self, direction=None):
		"""
		Updates the position of the bundler each time the machine moves. This method is called
		from the setpos in thMachine
		"""
		if direction==None: direction=pi/2.
		return getCartesian([self.s['dropPosJ'],pi/2.],origin=self.m.pos,direction=direction)
	
	def startTheBundler(self):
		"""
		Adds the time it takes for the driver to push the "start bundling"-button.
		"""
		return self.cmnd([],self.s['timeStartBundler'], auto=self.s['startBundler'])

	def bundleIt(self):
		"""
		Performs the actual bundling of the trees. 
		"""
		cB=self.currentBundle
		cB.length = 5 #cut in five meter long sections
		cB.xSection=sum([t.dbh**2 for t in cB.trees])
  		cB.biomass = sum([t.weight for t in cB.trees])#initial weight no losses when doing the bundles
   		cB.radius = sqrt(cB.length**2+(cB.diameter/2)**2)
		
	def bundlerFilled(self):
		"""
		controls when the bundler runs and makes a bundle.
		"""
		if self.currentBundle:
			if self.forceBundler==True:
				return True
			elif self.currentBundle.xSection > self.xSectionThresh: return True
			else: return False
		else: return False

	def resetBundle(self):
		self.forceBundler=False
		self.currentBundle=None

	def cmndWithDriver(self, commands, time):
		"""
		a method to set up the yield command, for use of the driver for a specific time.
		overrides superclass method

		priority not added here. If you want that, see how it's implemented in the same
		method for the planting machine.
		"""
		if self.usesDriver: #don't need to reserve driver..
			commands.extend([(hold, self, time)])
		else:
			commands.extend([(request, self, self.driver), (hold, self, time)])
			self.usesDriver=True
			switchTime=self.m.times['switchFocus']
			if self.driver.isIdle(): #check for how long he's been idle
				switchTime-=self.driver.idleTime()
				if switchTime<0: switchTime=0
			commands.extend([(hold, self, switchTime)]) #add time to switch focus
			commands.extend([(hold, self, time)])
			return commands

	def draw(self,ax):
		"""
		This is the drawing of the actual bundler without any trees in it
		"""
		
		#sets the nodes
		cart=getCartesian
		dx=1
		dy=1
		origin=self.pos
		a1=cart([dx,dy],origin=origin,fromLocalCart=True)
		a2=cart([-dx,dy],origin=origin,fromLocalCart=True)
		a3=cart([-dx,-dy],origin=origin,fromLocalCart=True)
		a4=cart([dx,-dy],origin=origin,fromLocalCart=True)
		a=[a1,a2,a3,a4]
		ax.add_patch(mpl.patches.Polygon(np.array(a), closed=True, facecolor=self.color))
