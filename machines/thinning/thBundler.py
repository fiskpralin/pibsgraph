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
		self.color='#CD0000'
		self.m=machine
		self.s=self.m.G.simParam
		self.pos=list(np.array(self.m.pos)+np.array([0,self.s['dropPosJ']]))
		self.timeBundle=self.s['timeBundle']
		self.maxXSection=self.s['maxXSectionJ']
		self.xSectionThresh=self.s['xSectionThreshJ']
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
			print 'Bundler runs'
			for c in self.startTheBundler(): yield c
			for c in self.cmnd([],self.s['timeBundle']-self.s['timeStartBundler'],auto=self.s['restOfBundling']): yield c
			for c in self.releaseDriver(): yield c
			self.bundleIt()
			self.dumpBundle()
			self.resetBundle()
			self.forceBundler=False
		
	def dumpBundle(self, direction=None):
		"""
		Releases the bundle at the current position. (And dumps the bundle in terrain)
		"""
		if direction is None: direction=pi/2

		#here the nodes of the bundle are set when the bundle is put in the terrain
		cB=self.currentBundle
		cB.pos=np.array(self.pos)+np.array([-2.5,0])#
		c1=getCartesian([-cB.diameter/2,cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		c2=getCartesian([-cB.diameter/2, -cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		c3=getCartesian([cB.diameter/2, -cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		c4=getCartesian([cB.diameter/2,cB.length/2], origin=cB.pos, direction=direction, fromLocalCart=True)
		cB.nodes=[c1,c2,c3,c4]
		
		self.m.G.terrain.piles.append(cB)#adds the pile to the list of piles in terrain
		self.m.G.terrain.addObstacle(cB)
		print '*SAVED the bundle with',len(self.currentBundle.trees),'trees at pos:',cB.pos

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
		cB.xSection=sum([cB.getXSection(tree=t) for t in cB.trees])
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
		This is the drawing of the actual bundler without any trees in it. Oh come on
		you can do a nicer bundler than this!
		"""
		cart=getCartesian
		origin=self.pos
		#sets the outer edges
		width=1.2
		back=-0.2
		front=2.8
		off=0.3
		a1a=cart([width/2,front-off],origin=origin,fromLocalCart=True)
		a1b=cart([width/2-off,front],origin=origin,fromLocalCart=True)
		a2a=cart([-width/2+off,front],origin=origin,fromLocalCart=True)
		a2b=cart([-width/2,front-off],origin=origin,fromLocalCart=True)
		a3a=cart([-width/2,back+off],origin=origin,fromLocalCart=True)
		a3b=cart([-width/2+off,back],origin=origin,fromLocalCart=True)
		a4a=cart([width/2-off,back],origin=origin,fromLocalCart=True)
		a4b=cart([width/2,back+off],origin=origin,fromLocalCart=True)
		a=[a1a,a1b,a2a,a2b,a3a,a3b,a4a,a4b]
		ax.add_patch(mpl.patches.Polygon(np.array(a), closed=True, facecolor=self.color))

		#draw connection and transmission line
		allback=-0.4
		trw=0.2
		t1=cart([trw/2,allback/2],origin=origin,fromLocalCart=True)
		t2=cart([-trw/2,allback/2],origin=origin,fromLocalCart=True)
		t3=cart([-trw/2,allback],origin=origin,fromLocalCart=True)
		t4=cart([trw/2,allback],origin=origin,fromLocalCart=True)
		t=[t1,t2,t3,t4]
		ax.add_patch(mpl.patches.Polygon(np.array(t), closed=True, facecolor='k'))

		#draw the tyres of the bundler
		ww=0.2 #width of the tyres
		wr=0.45 #radius of the tyres
		origin1=[origin[0]+width/2+ww/2,origin[1]+(front-off-wr)]
		origin2=[origin[0]-width/2-ww/2,origin[1]+(front-off-wr)]
		origin3=[origin[0]-width/2-ww/2,origin[1]+(front-off-3*wr-0.2)]
		origin4=[origin[0]+width/2+ww/2,origin[1]+(front-off-3*wr-0.2)]

		w11=cart([ww/2,wr],origin=origin1,fromLocalCart=True)
		w12=cart([-ww/2,wr],origin=origin1,fromLocalCart=True)
		w13=cart([-ww/2,-wr],origin=origin1,fromLocalCart=True)
		w14=cart([ww/2,-wr],origin=origin1,fromLocalCart=True)

		w21=cart([ww/2,wr],origin=origin2,fromLocalCart=True)
		w22=cart([-ww/2,wr],origin=origin2,fromLocalCart=True)
		w23=cart([-ww/2,-wr],origin=origin2,fromLocalCart=True)
		w24=cart([ww/2,-wr],origin=origin2,fromLocalCart=True)

		w31=cart([ww/2,wr],origin=origin3,fromLocalCart=True)
		w32=cart([-ww/2,wr],origin=origin3,fromLocalCart=True)
		w33=cart([-ww/2,-wr],origin=origin3,fromLocalCart=True)
		w34=cart([ww/2,-wr],origin=origin3,fromLocalCart=True)
		
		w41=cart([ww/2,wr],origin=origin4,fromLocalCart=True)
		w42=cart([-ww/2,wr],origin=origin4,fromLocalCart=True)
		w43=cart([-ww/2,-wr],origin=origin4,fromLocalCart=True)
		w44=cart([ww/2,-wr],origin=origin4,fromLocalCart=True)
		w1=[w11,w12,w13,w14]
		w2=[w21,w22,w23,w24]
		w3=[w31,w32,w33,w34]
		w4=[w41,w42,w43,w44]
		for w in [w1,w2,w3,w4]:
			ax.add_patch(mpl.patches.Polygon(np.array(w), closed=True, facecolor='k'))
		"""	
		#draw the claws and the ring
		widthring=0.6
		widthclaw=0.8
		thick=0.0025

		w1=cart([widthring/2,thick+front-1.5*wr],origin=origin,fromLocalCart=True)
		w2=cart([-widthring/2,thick+front-1.5*wr],origin=origin,fromLocalCart=True)
		w3=cart([-widthring/2,-thick+front-1.5*wr],origin=origin,fromLocalCart=True)
		w4=cart([widthring/2,-thick+front-1.5*wr],origin=origin,fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([w1,w2,w3,w4]), closed=True, facecolor='k'))

		c1=cart([widthclaw/2,thick+front-4*wr],origin=origin,fromLocalCart=True)
		c2=cart([-widthclaw/2,thick+front-4*wr],origin=origin,fromLocalCart=True)
		c3=cart([-widthclaw/2,-thick+front-4*wr],origin=origin,fromLocalCart=True)
		c4=cart([widthclaw/2,-thick+front-4*wr],origin=origin,fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k'))
		
		y1=cart([widthclaw/2,thick+front-5*wr],origin=origin,fromLocalCart=True)
		y2=cart([-widthclaw/2,thick+front-5*wr],origin=origin,fromLocalCart=True)
		y3=cart([-widthclaw/2,-thick+front-5*wr],origin=origin,fromLocalCart=True)
		y4=cart([widthclaw/2,-thick+front-5*wr],origin=origin,fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([y1,y2,y3,y4]), closed=True, facecolor='k'))
		"""
