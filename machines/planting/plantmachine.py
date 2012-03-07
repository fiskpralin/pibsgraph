from SimPy.Simulation  import *
import machines
from machines.basics import Machine, UsesDriver, Operator
from heads import PlantHead, Mplanter, Bracke
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
from matplotlib.patches import Circle, Rectangle 
import copy
import time
from math import *
"""This module stores all the machines."""		
###################################################
# The planting machine
##################################################
class PlantMachine(Machine):
	"""
	The planting machine. A Volvo EC210C. Has one or two planting devices which in turn have two planting heads each. 
This machine is not really intelligent, the sofisticated behavious is programmed in the planting devices. Machine does not move.
	"""
	def __init__(self, name, sim, G, mtype='1a2h', craneLim=None):
		if not craneLim: craneLim=[4.0,9.0]
		Machine.__init__(self, name, sim, G=G, mass=21000)
		self.driver=Operator(sim=self.sim, delay=10000) #does not go on breaks..
		self.sim.activate(self.driver, self.driver.work())
		self.type=mtype
		if self.type=='1a2h' or self.type=='2a4h':
			self.headType='Mplanter'
		else:
			self.headType='Bracke'
		self.pos=[0,0]
		self.craneMaxL=G.craneLim[1]
		self.craneMinL=G.craneLim[0]
		if self.craneMaxL <= self.craneMinL: raise Exception('crane maximum is smaller than crane minimum...')
		self.pos[0]=random.uniform(G.terrain.xlim[0]+self.craneMaxL, G.terrain.xlim[1]- self.craneMaxL)
		self.pos[1]=random.uniform(G.terrain.ylim[0]+self.craneMaxL, G.terrain.ylim[1]- self.craneMaxL)
		if self.G.PMattach:
			self.craneIntersect=self.G.PMattach
		else:
			self.craneIntersect=3.0
		if self.craneMinL<=self.craneIntersect: self.craneMinL=self.craneIntersect-0.01 # of course..
		self.visual={'upperStructLength':4.3, 'upperStructWidth':2.540 ,'trackWidth':0.6,'trackLength':4.460 }
		po1=[self.visual['upperStructWidth']/2.+(2.990-2.540)/4., 0]
		po2=[-self.visual['upperStructWidth']/2.-(2.990-2.540)/4., 0]
		self.visual['trackCntrPosLoc']=[po1,po2] #just for plotting..
		self.workingArea=pow(self.craneMaxL,2)*pi/2.-pow(self.craneMinL,2)*pi/2.
		if self.G.PMstockingRate:
			self.stockingRate=self.G.PMstockingRate
		else:
			self.stockingRate=2000 # plants/ha
		self.plantMinDist=G.plantMinDist
		self.dibbleDepth=0.1
		self.nSeedlingsPWArea=floor(self.stockingRate/10000.*self.workingArea)
		print "sPerWorkarea:", self.nSeedlingsPWArea, "cranemax:", self.craneMaxL, "cranemin:",self.craneMinL, "attach:", self.craneIntersect
		self.trees=0 	#trees in trunk
		#self.direction=random.uniform(0,2*pi)
		self.direction=0
		self.times={'diggTime': 3, 'heapTime': 2,'moundAndHeapTime': 5, 'dibbleDownTime': 1, 'relSeedlingTime': 2, 'haltTime': 3, 'searchTime': 2, 'switchFocus':0}
		#if self.G.PMfocusSwitch: self.times['switchFocus']= self.G.PMfocusSwitch
		if self.headType=='Bracke':
			self.times['searchTime']=0 #specifics for this head..
		self.timeConsumption={'diggTime': 0, 'heapTime': 0,'moundAndHeapTime': 0, 'dibbleDownTime': 0, 'relSeedlingTime': 0, 'haltTime': 0, 'searchTime': 0, 'switchFocus':0, 'machineMovement':0}
		"""
		updated:
		-searchTime
		-switchFocus
		"""
		self.type=mtype
		self.pDevs=[]
		self.treesPlanted=[]
		self.automatic=G.automatic
		self.velocities={'machine': 0.5, 'angMachine':11.8*2*pi/60., 'angCranes': 15*pi/180, 'radial': 1.6} #radians/sec and m/s
		self.timeConstants={'machine': 5, 'maincrane':1.5, 'subcrane':0.1} #time taken to initiate movements.
		if self.G.PMradialCraneSpeed: self.velocities['radial']=self.G.PMradialCraneSpeed
		self.angleLim=self.G.PMangleLim #angle limit between the two arms/planting devices.
		self.treeMoni=Monitor(name="Trees planted")
		self.leftAngleMoni=Monitor(name="angle for left crane")
		self.rightAngleMoni=Monitor(name="angle for right crane")
		self.treeMoni.observe(len(self.treesPlanted), self.sim.now())
		p1=PlantingDevice('rightDevice', sim=self.sim, belongToMachine=self, G=self.G)
		self.sim.activate(p1,p1.run())
		self.pDevs.append(p1)
		if mtype[0:2]=='2a': #add another one
			self.times['switchFocus']=2
			self.timeConstants['machine']*=1.5 #longer time for 2a
			self.velocities['machine']*=0.75
			p2=PlantingDevice('leftDevice', sim=self.sim, belongToMachine=self, G=self.G)
			self.sim.activate(p2,p2.run())
			self.pDevs.append(p2)
			if self.exceedsAngleLim(self.pDevs[0], self.pDevs[0].pos, self.pDevs[1]):
				#angle is too big, needs to adjust.
				it=0
				while self.exceedsAngleLim(self.pDevs[0], self.pDevs[0].pos, self.pDevs[1]):
					newcyl=[self.pDevs[0].posCyl[0], self.pDevs[0].posCyl[1]+pi/50.]
					print self.pDevs[0].posCyl, newcyl
					newpos=getCartesian(newcyl, origin=self.pos, direction=self.direction)
					if collide(self.pDevs[0], self.pDevs[1], o1pos=newpos) or it>20:
						raise Exception('exceeds initially and cannot adjust.', it, newcyl)
					self.pDevs[0].setPos(newpos)				
			self.mass+=4000. #the other planting head has a mass of 4 tons.
		self.inPlaceEvent=SimEvent('machine has moved in place', sim=self.sim) #fired when machine has moved
		self.calcStumpsInWA() #updates some statistics
	def run(self): #the method has to be here in order to be counted as an entity
		#get machine in place. Assumes that machine comes from last position in half-cricle pattern.
		distance=self.craneMaxL #not exact, half circle may overlap more or less than this.
		time=self.timeConstants['machine']+distance/self.velocities['machine']
		yield request, self, self.driver
		yield hold, self, time
		yield release, self, self.driver
		self.timeConsumption['machineMovement']=time
		self.inPlaceEvent.signal()
		while True:
			yield hold, self, 1
			self.stopControl()
	def exceedsAngleLim(self, p1in, pos1in,p2in, pos2in='undefined'):
		"""move p1 to pos1, do we exceed the angular limits and do the cranes intersect?
		i.e is it possible to place the intersection point of the two small cranes at a position so that the angle does not exceed the limit.
		Also checks if point is too far for crane to reach."""
		tic=time.clock()
		if pos2in == 'undefined': pos2in=p2in.pos
		if p1in.mountPoint == 'left' and p2in.mountPoint == 'right':
			p1=p1in
			pos1=pos1in
			p2=p2in
			pos2=pos2in
		elif p1in.mountPoint == 'right' and p2in.mountPoint == 'left':
			p1=p2in
			pos1=pos2in
			p2=p1in
			pos2=pos1in
		else:
			raise Exception("ERROR, exceedsAngleLim only defined for 2a with names 'left' and 'right'")
		#p1 is to the left, p2 to the right..
		#some first checks: if we could classify as exceeds in this stage, a lot of time is saved.
		D=getDistance(pos1, pos2)
		p1cyl=self.getCylindrical(pos1)
		p2cyl=self.getCylindrical(pos2)
		if p2cyl[1] <=p1cyl[1]: alpha=abs(self.angleLim[1]) #no crossing.
		else: alpha=abs(self.angleLim[0]) #cranes cross!
		if abs(p1cyl[1]-p2cyl[1])>alpha:
			PlantingDevice.timesProf[3]+=time.clock()-tic	
			return True #there is no way that the angle that counts can be smaller than this angle. Think about it.
		if alpha<pi/2. and alpha>0.4636: #pretty long derivation, number comes from arctan(0.5)...
			rmax=max(p1cyl[0], p2cyl[0])
			if D/2.>(rmax-self.craneIntersect)*tan(alpha):
				PlantingDevice.timesProf[3]+=time.clock()-tic	
				return True #saves a lot of time, but does not cover all the cases
		#algorithm: test angles from pi to 0.. brute force and time consuming, and solutions could be missed.
		if collide(o1=p1in, o2=p2in, o1pos=pos1in, o2pos=pos2in): 
			PlantingDevice.timesProf[3]+=time.clock()-tic
			return True #collides..
		a=self.getAngles(pos1,pos2)
		PlantingDevice.timesProf[3]+=time.clock()-tic	
		return a	
	def getAngles(self, pos1, pos2, optimize=False, dth=pi/200):
		"""if optimize, this function returns the angles at the position of smallest angles. otherwise it checks if this position is possible.
		function assumes that there is two cranes."""
		th=0
		lim=self.angleLim
		ang=[0,0]
		r=[0,0]
		angmin=1000
		angCrane=th
		rI=self.craneIntersect
		#determine angle max/min span:
		direction=self.direction
		[r1,th1]=self.getCylindrical(pos1) 
		[r2,th2]=self.getCylindrical(pos2)
		#this gives us some marginal. More analysis can be performed to optimize, analytical solution might even exist.
		thmin=min(th1, th2)
		thmax=max(th1, th2)
		if lim[0]<0:
			thmin-=pi/7.
			thmax+=pi/7.		
		span=thmax-thmin
		thMiddle=min(th1,th2)+(max(th1,th2)-min(th1,th2))/2.
		if thmin<-abs(lim[0]): thmin=-abs(lim[0])
		if thmax>pi+abs(lim[0]): thmax=pi+abs(lim[0])
		angles=[thMiddle]+list(np.arange(thMiddle-span/4., thMiddle+span/4., dth))+ \
			   list(np.arange(thmin, thMiddle-span/4. ,dth))+list(np.arange(thMiddle+span/4., thmax, dth))		
		for th in angles: #scan through the possible angles for the crane, look for angle exceeds.
			spreadPoint=self.getCartesian([rI, th]) 
			direction=self.direction+th-pi/2. #direction of boom
			[r1,th1]=self.getCylindrical(pos1, origin=spreadPoint, direction=direction) 
			[r2,th2]=self.getCylindrical(pos2, origin=spreadPoint, direction=direction)
			th1=th1-pi/2.
			th2=pi/2.-th2
			if lim[0]<th1<lim[1] and lim[0]<th2<lim[1] and rI+r1<self.craneMaxL and rI+r2<self.craneMaxL:
				#angles agree and cranes are not too extended. If craneIntersect+remaineing crane > craneMax, too extended.
				if optimize:
					angle=abs(th1)+abs(th2)
					if angle<angmin:
						angmin=angle
						ang=[th1,th2]
						r=[r1,r2]
						angCrane=th
				else:
					return False
			elif th1>lim[1] and th2>lim[1] and not optimize: #positive angle means point is in between. If this point exist, a solution can impossibly be found.
				return True
		if optimize:	
			if angmin is not 1000:
				return ang+[angCrane]+r
			else:
				#this happens very rarely.. but ..
				if dth < pi/4000.: #to avoid infinite loop.. below return has "spun" several times..
					raise Exception("ERROR, getAngles:  the current position is not allowed, exceeds angle limits %f, %f............%f,%f"%(pos1[0], pos1[1], pos2[0], pos2[1]))
				return self.getAngles(pos1, pos2, optimize=optimize, dth=dth/5.)				
		else:
			return True #angles exceed..
		#assume that p1 is left and p2 right.
	def stopControl(self):
		"""Checks if simulations should be stopped"""
		reason=None
		print "trees:", self.G.simParam['planted'], "area:", self.G.simParam['areaCovered']
		print (len(self.treesPlanted)+self.G.simParam['planted'])/(self.G.simParam['areaCovered']+self.workingArea), self.stockingRate/10000.0
		if self.driver.resting:
			return False #wait until he's not at rest, should not result in an infinite loop.
		elif (len(self.treesPlanted)+self.G.simParam['planted'])/(self.G.simParam['areaCovered']+self.workingArea)>=self.stockingRate/10000.0:
			#last is to ensure that machine is above productivity maximum
			reason="the desired no. of trees are planted"
		elif len(self.pDevs)==1 and self.pDevs[0].noMoreSpots:
			reason="pDev out of spots."
		else:
			reason="plantDevs are passive and so are all the plantheads."
			for p in self.pDevs:
				if not p.passive():
					return False
				else:
					for pH in p.plantHeads:
						if pH.cause is not None: return False #machine is waiting for
			print "Stop Reason:", reason
		self.sim.stopSimulation()
	def isWithinPlantingBorders(self, pos, c='cartesian'):
		#plantingarea is approximated as circle.
		w=self.pDevs[0].plantAreaW #2.5 def
		L=self.pDevs[0].plantAreaL # 1 def
		l=self.pDevs[0].plantHeads[0].length
		#transform to cylindrical if necessary
		if c=='cartesian' or c=='Cartesian':
			cyl=self.getCylindrical(pos)
		elif c=='cylindrical' or c=='Cylindrical':
			cyl=pos
		else:
			raise Exception("error, isWithingPlantingBorders did not recognize %s"%c)
		r=cyl[0]
		th=cyl[1]
		#is radius too small or too big?
		if r<self.craneMinL+L-l/2 or r > self.craneMaxL-l/2.:
			return False
		if self.headType=='Mplanter': thInner=asin((w/2.)/r)
		else: thInner=0
		#is angle too small or too big?
		if th>pi-thInner or th<thInner:
			return False
		return True
	def calcStumpsInWA(self):
		"""
		required statistics. Roots are included
		"""
		#make a polygon that approximates out half circle
		cart=getCartesian
		cyl=getCylindrical
		polynodes=[]
		Lmin=self.craneMinL
		Lmax=self.craneMaxL
		points=15 #how many point in a half circle arc
		thlim=[0,pi]
		for th in np.linspace(thlim[0], thlim[1],points):
			polynodes.append(cart([Lmax,th], direction=self.direction, origin=self.pos))
		for th in np.linspace(thlim[1], thlim[0],points):
			polynodes.append(cart([Lmin,th], direction=self.direction, origin=self.pos))
		pol=Polygon(self.pos, polynodes)
		potentialStumps=self.G.terrain.GetStumps(self.pos, self.craneMaxL)
		sno=0
		sdiam=0
		for s in potentialStumps:
			if collide(s,pol):
				sno+=1
				sdiam+=s.dbh
		self.sim.stats['stumps in WA']=sno
		self.sim.stats['stumps in WA sum diamater']=sdiam
	def draw(self, ax):
		cart=self.getCartesian
		v=self.visual
		#plot the machine tracks.
		p1=v['trackCntrPosLoc'][0]
		p2=v['trackCntrPosLoc'][1]
		for p in [p1,p2]:
			verts=[]
			codes=[]
			c1=cart([p[0]+v['trackWidth']/2., p[1]+v['trackLength']/2.],origin=self.pos, direction=self.direction, local=False, fromLocalCart=True)
			c2=cart([p[0]-v['trackWidth']/2., p[1]+v['trackLength']/2.],origin=self.pos, direction=self.direction, local=False, fromLocalCart=True)
			c3=cart([p[0]-v['trackWidth']/2., p[1]-v['trackLength']/2.],origin=self.pos, direction=self.direction, local=False, fromLocalCart=True)
			c4=cart([p[0]+v['trackWidth']/2., p[1]-v['trackLength']/2.],origin=self.pos, direction=self.direction, local=False, fromLocalCart=True)
			ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k'))
		#plot the machine
		if len(self.pDevs)>1:
			a=self.getAngles(self.pDevs[1].pos, self.pDevs[0].pos, optimize=True) #angles not allowed may accur during movements.
			craneDir=self.direction-pi/2.+a[2]
		else:
			a=[0,0,self.direction] #first two are not relevant for 1a
			craneDir=self.direction+self.pDevs[0].posCyl[1]-pi/2.
		D=2.850 #yposition of back
		front=v['upperStructLength']-D #y-position of front
		c1=cart([v['upperStructWidth']/2., front],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
		c2=cart([-v['upperStructWidth']/2., front],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
		c3=cart([-v['upperStructWidth']/2., -D],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
		c4=cart([v['upperStructWidth']/2., -D],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
		ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='y'))
		#plot cranes
		alpha=1
		if len(self.pDevs)>1:
			#first, plot the first part of the crane:
			L=self.craneIntersect
			W=0.3 #3dm
			c1=cart([W/2., 0],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			c2=cart([W/2., L],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			c3=cart([-W/2., L],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			c4=cart([-W/2., 0],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k', alpha=alpha))
			#add the "bar" at the point of separation, let the separation be 0.7m
			origin=cart([0, L],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			L=W
			W2=0.7
			c1=cart([W2/2., -L/2.],origin=origin, direction=craneDir, local=False, fromLocalCart=True)
			c2=cart([W2/2., L/2.],origin=origin, direction=craneDir, local=False, fromLocalCart=True)
			c3=cart([-W2/2., L/2.],origin=origin, direction=craneDir, local=False, fromLocalCart=True)
			c4=cart([-W2/2., -L/2.],origin=origin, direction=craneDir, local=False, fromLocalCart=True)
			ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k', alpha=alpha))
			#add the two outer cranes:
			W=W/2.
			orig1=cart([W2/2., 0],origin=origin, direction=craneDir, local=False, fromLocalCart=True) #left
			orig2=cart([-W2/2., 0],origin=origin, direction=craneDir, local=False, fromLocalCart=True) #right
			pos1=self.pDevs[0].pos #left
			pos2=self.pDevs[1].pos	#right
			for p in [(orig1, pos1), (orig2, pos2)]:
				L=getDistance(p[0],p[1])
				[r,th]=self.getCylindrical(p[1], origin=p[0], direction=craneDir)
				#redefine cartesian, use direction th.
				direction=craneDir-pi/2.+th
				c1=cart([W/2., 0],origin=p[0], direction=direction, local=False, fromLocalCart=True)
				c2=cart([W/2., L],origin=p[0], direction=direction, local=False, fromLocalCart=True)
				c3=cart([-W/2., L],origin=p[0], direction=direction, local=False, fromLocalCart=True)
				c4=cart([-W/2., 0],origin=p[0], direction=direction, local=False, fromLocalCart=True)
				ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k', alpha=alpha))
		else:
			L=self.pDevs[0].posCyl[0]
			W=0.3 #3dm
			c1=cart([W/2., 0],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			c2=cart([W/2., L],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			c3=cart([-W/2., L],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			c4=cart([-W/2., 0],origin=self.pos, direction=craneDir, local=False, fromLocalCart=True)
			ax.add_patch(mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k', alpha=alpha))
			

###################################################
## Planting Device
###################################################				
class PlantingDevice(Process, Obstacle, UsesDriver):
	"""This is the end of the crane, consists  of two plantingheads that do not move in 
	relation to each other (they could in the z-direction..)"""
	timesProf=[0,0,0, 0] #movement, plant, 
	def __init__(self, name, sim, belongToMachine,G):
		Process.__init__(self, name, sim)
		UsesDriver.__init__(self, belongToMachine.driver)
		Obstacle.__init__(self, [0,0], isSpherical=False, terrain=G.terrain)
		self.m=belongToMachine
		self.G=G
		#cylindrical coordinates in relation to machine:
		#crane starts in it minimum position to the left of the driver.
		#algorithms for planting:
		self.noMoreSpots=False #used to cancel the search for new spots
		self.idealSpots=[]
		self.pSpots=[]
		self.plantHeads=[]
		self.otherDevice = None #defined later if 2a
		self.struckLastTime=False
		self.timeConsumption={'crane movement':0}
		self.moveEvent=SimEvent(name='planthead moves', sim=self.sim)
		if len(self.m.pDevs)==0: #first arm, right, could also be 1a
			self.mountPoint='right'	
		elif len(self.m.pDevs)==1:
			self.mountPoint='left'
		else:
			raise Exception("ERROR: cannot mount more than two planting devices to machine.%f"%len(self.m.pDevs))
		if self.m.headType=='Mplanter':
			self.plantSepDist=self.G.PMplantSepDist
			for lr in ["left", "right"]:
				h=Mplanter(self.name+"_"+lr+"_head", self.sim, PD=self,leftRight=lr)
				self.sim.activate(h,h.run())
				self.plantHeads.append(h)
				self.plantAreaW=self.plantSepDist+h.width
		elif self.m.headType=='Bracke':
			h=Bracke(self.name+"_"+"brackeHead", self.sim, PD=self)
			self.sim.activate(h, h.run())
			self.plantHeads.append(h)
			self.plantAreaW=h.width
		else:
			raise Exception('Headtype not supported: %s'%str(self.headType))
		self.plantAreaL=2*h.length
		self.radius=0.5*sqrt(self.plantAreaW**2+self.plantAreaL**2) #needed for plantingdevice to be regarded as obst.
		#define the optimal nodes:
		self.initOpt()
		self.setPos(self.optNodes[0])
		self.optNode=0
		self.autoMoved=True
		self.sim.machines.append(self)
		self.pSpots.append(self.pos)
		self.plantSignals=0
	def run(self):
		#find the other device:
		yield waitevent, self, self.m.inPlaceEvent
		if len(self.m.pDevs)==2:
			devices=self.m.pDevs
			for dev in devices:
				if dev is not self:
					self.otherDevice=dev
		#main loop:
		while True:
			for c in self.autoMove():
				self.debugPrint('automoves.')
				yield c #conditions are inside automove, if not return []
			(commands, plant)=self.moveToNext()
			for c in commands: yield c
			if plant: #if a possible plantingspot was found, start planting process
				for c in self.plant(): 
					if c[0]=="interrupt": #starts planting for the pHeads.
						for pHead in c[1]:
							self.debugPrint('interupts the plantheads')
							self.interrupt(pHead)
						yield hold, self, 0.000000001 #timing, let pHead have time to get in queue for driver
						for cm in self.releaseDriver(): yield cm
					else:
						self.debugPrint('executes commmands given by plant, but does not wait for heads.')
						yield c
				self.debugPrint('done planting for this loop')
			self.m.stopControl()
			if self.noMoreSpots and not plant: 
				for c in self.releaseDriver(): yield c
				yield passivate, self #wait for other head 
			if self.lastPos==self.pos:
				self.struckLastTime=True
			else:
				self.struckLastTime=False
	def initOpt(self):
		"""sets the pattern for optimal positions."""
		self.optNodes=[]
		self.optNode=-1
		if self.m.headType=='Bracke':
			bracke=True #one head per decice
		else:
			bracke=False
		if self.m.type[0:2]=='2a':
			#this is strictly for 2000 plants/ha, i.e 10 spots per half circle and [4,9]m crane dimensions
			w1 = 1.3
			w2 = 1.0
			if self.mountPoint is 'left':
				for r in [self.m.craneMaxL-w2, self.m.craneMinL+w2]:
					th=pi-asin(w1/r)
					self.optNodes.append(self.m.getCartesian([r,th]))
					self.idealSpots.append(self.m.getCartesian([r,th]))
				dth=(pi-2*asin(w1/r))/3.
				th-=dth
				self.optNodes.append(self.m.getCartesian([r,th]))
				self.idealSpots.append(self.m.getCartesian([r,th]))
				r=self.m.craneMaxL-w2
				th=pi-asin(w1/r)
				dth=(pi-asin(w1/r))/5. #outer			
				th-=3*dth
				for th in [th, th-dth]:
					self.optNodes.append(self.m.getCartesian([r,th]))
					self.idealSpots.append(self.m.getCartesian([r,th]))
			else:
				r=self.m.craneMaxL-w2
				th=pi-asin(w1/r)
				dth=(pi-asin(w1/r))/5. #outer
				th-=dth
				for th in [th, th-dth]:
					self.optNodes.append(self.m.getCartesian([r,th]))
					self.idealSpots.append(self.m.getCartesian([r,th]))
				r=self.m.craneMinL+w2
				dth=(pi-2*asin(w1/r))/3.
				th=pi-asin(w1/r)-2.*dth
				for th in [th, th-dth]:
					self.optNodes.append(self.m.getCartesian([r,th]))
					self.idealSpots.append(self.m.getCartesian([r,th]))
				r=self.m.craneMaxL-w2
				th=asin(w1/r)
				self.optNodes.append(self.m.getCartesian([r,th]))
				self.idealSpots.append(self.m.getCartesian([r,th]))
		elif self.m.type[0:2]=='1a':
			w1 = self.plantAreaW/2.
			w2 = self.plantAreaL/2.
			if isinstance(self.plantHeads[0], Bracke):
				spaceMin=self.m.plantMinDist
			else:
				spaceMin=self.plantSepDist+self.m.plantMinDist #minimum spacing for angular movements.
			n=ceil(self.m.nSeedlingsPWArea/2.) #/2 due to two plantHeads per device
			nLeft=n
			lInner = (self.m.craneMinL+w2)*(pi-2*asin(w1/(self.m.craneMinL+w2)))
			sLength = sqrt(pow(self.m.craneMaxL-w2,2)-pow(w1,2))-sqrt(pow(self.m.craneMinL+w2,2)-pow(w1,2))
			lOuter =(self.m.craneMaxL-w2)*(pi-2*asin(w1/(self.m.craneMaxL-w2)))
			lMiddle=0
			rList=[self.m.craneMinL+w2, 'border', self.m.craneMaxL-w2]
			lTot=lInner+sLength+lOuter
			rMiddle=-1
			dr=self.m.craneMaxL-w2-(self.m.craneMinL+w2)
			if dr>2*self.m.plantMinDist: #create another sweep
				rMiddle=(self.m.craneMaxL-w2)-dr/2.
				lMiddle=rMiddle*(pi-2*asin(w1/rMiddle))
				rList.append(rMiddle)
				lTot+=lMiddle
			lCurr=0
			for r in rList:
				if r is 'border':
					r=self.m.craneMinL+w2
					L=sLength
					nSection=nLeft*(L/(lTot-lCurr))
					#dr=(L-2*dr)/nSection =>
					dr=L/(nSection+2.)
					if dr<self.m.plantMinDist: dr=self.m.plantMinDist
					a=0
					while r<(self.m.craneMaxL-w2)-2*dr:
						r+=dr
						th=asin(w1/(r))
						self.optNodes.append(self.m.getCartesian([r,th]))
						self.idealSpots.append(self.m.getCartesian([r,th]))
						a+=1
				else:
					L=r*(pi-2*asin(w1/r))
					nSection=nLeft*(L/(lTot-lCurr)) #how much to plant on this section
					dth=(pi-2*asin(w1/r))/nSection
					if dth*r < spaceMin: dth=spaceMin/r
					if r == self.m.craneMinL+w2 or r==rMiddle:
						dth=-dth
						th=pi-asin(w1/(r))
					else:
						th=asin(w1/(r))
					a=0
					while abs(th-pi/2.)-0.00001<=(pi-2*asin(w1/r))/2.:
						self.optNodes.append(self.m.getCartesian([r,th]))
						self.idealSpots.append(self.m.getCartesian([r,th]))
						th+=dth
						a+=1
				if a<nSection: #if spaceMin got into it and interfered.
					nSection=a
				nLeft-=nSection
				lCurr+=L
		else:
			raise Exception("ERROR, type %s not supported"%self.type)
		if bracke: #take every plantingpoint...
			o=self.optNodes
			onew=[]
			i=self.idealSpots
			inew=[]
			c=self.m.getCartesian
			w=2.01/2.
			lastPos=self.pos
			for p in self.optNodes:
				#p1 on the left, p2 on the right
				cyl=self.m.getCylindrical(p)
				direction=self.m.direction+cyl[1]-pi/2.
				p1=c([-w, 0], origin=p, direction=direction, fromLocalCart=True)
				p2=c([w, 0], origin=p, direction=direction, fromLocalCart=True)
				#make the order right.
				if getDistance(p1,lastPos)<getDistance(p2, lastPos):
					plist=[p1,p2]
				else:
					plist=[p2,p1]
				for ptmp in plist:
					onew.append(ptmp)
					inew.append(ptmp)
				lastPos=ptmp
			self.optNodes=onew
			self.idealSpots=inew
	def debugPrint(self, string):
		print self.sim.now(), self.name, string
	def autoMove(self):
		if self.driverBusy() and self.m.automatic['automove']==True and not self.autoMoved:#if driver is busy, move to next ideal automatically
			self.autoMoved=False
			if self.noMoreSpots:
				return []
			ideal=self.getNextOptimal() #takes care of exceeds and so on.
			if ideal is not self.pos:
				traveltime=self.setPos(ideal)
				self.autoMoved=True
				return self.cmnd([], traveltime, auto=True)
		return []
	def moveToNext(self):
		"""identify the next spatially good position. An algorithm based on minimal crane extractions is used. Returns a boolean plant value and the yield command, the plant value indicates if a good position was found. If so, the position is already updated. and is used as the optimal next pos."""
		if self.G.debug:
			tic=time.clock()
		self.debugPrint('looks for new spot')
		exceeds=self.m.exceedsAngleLim	#function
		inside=self.m.isWithinPlantingBorders	#function
		cart=self.m.getCartesian
		auto=self.m.automatic
		t=self.m.times
		commands=[]
		if self.autoMoved:
			opt=self.pos
			self.autoMoved=False #if this search is unsuccessfull, automove is enabled to next ideal pos.
		else:
			opt=self.getNextOptimal()
		moveTo=opt #for so long..
		rTemp=0.1
		thTemp=0
		b=0.05 #constant for the spiral
		a=0.1
		plant=True #we will plant in this step...
		d2=self.m.plantMinDist**2 #dist^2
		possible = False #for so long
		while not possible:
			tic=time.clock()
			possible=True
			obstList=self.G.terrain.GetVisibleObstacles(moveTo, R=self.radius)
			treeList=self.G.terrain.GetTrees(moveTo, R=self.radius+self.m.plantMinDist)
			obstList+=[tr for tr in treeList if not tr in obstList] #this procedure minimizes R in Getobst
			#[p1, p2]=self.getPHCoord(moveTo)
			phPos=self.getPHCoord(moveTo)
			plantSpots=self.getPlantingCoord(moveTo)
			#[f1,f2]=self.getPlantingCoord(moveTo)
			if self.otherDevice is not None:
				otherDevPlantCor=self.otherDevice.getPlantingCoord(self.otherDevice.pos)
				#check for colissions and similar related to other device
				if collide(self, self.otherDevice, o1pos=moveTo): 
					possible=False
				else:
					for o in otherDevPlantCor:
						for f in plantSpots:
							#if getDistanceSq(f1, o)<d2 or getDistanceSq(f2, o)<d2:
							if getDistanceSq(f,o)<d2:#plantingspot of device is closer than allowed to other Device's plantingspot
								possible=False
								break		
			if possible:	#either 1a or angle OK and above check OK
				for obst in obstList:
					#tic=time.clock()
					if isinstance(obst, Tree):
						#other demands, more than 1.5 m from plantingspot.
			   			for f in plantSpots:
							#if getDistanceSq(f1, o)<d2 or getDistanceSq(f2, o)<d2:
							if getDistanceSq(f, obst.pos)<d2 or collide(self, obst, o1pos=moveTo):
								possible=False
								break
					elif isinstance(obst, Hole): #hole can be in beetween plantheads... Plantpos can be in hole.
						if len(self.plantHeads)==1: #bracke
							possible=False
							break
						elif collide(self.plantHeads[0], obst, o1pos=phPos[0]) or collide(self.plantHeads[1], obst, o1pos=phPos[1]):
							possible=False
							#PlantingDevice.timesProf[0]+=time.clock()-tic	
							break
					elif collide(self, obst, o1pos=moveTo):
						possible=False
						#PlantingDevice.timesProf[0]+=time.clock()-tic	
						break
				if possible and self.otherDevice is not None and exceeds(self, moveTo, self.otherDevice):
					possible=False	#angle is too big to the other device
			#at this point, all test for "possibility" are performed.
			PlantingDevice.timesProf[0]+=time.clock()-tic
			dthini=pi/50.
			if not possible:
				#move in a spiral outwards
				rTemp=a+b*thTemp
				dth=(pi/25.)/(rTemp/2.)
				thTemp+=dth
				thInit=thTemp #used to avoid infinite loop
				moveTo=cart([rTemp,thTemp],opt)
				while not inside(moveTo) or (self.otherDevice is not None and exceeds(self, moveTo, self.otherDevice)):
					#outside borders or too big angle.. make above expression shorter..
					#self.pSpots.append(self.m.getCartesian([rTemp,thTemp], opt))
					rTemp=a+b*thTemp
					thTemp+=(pi/25.)/(rTemp/2.)					
					#if abs(thTemp-thInit)>2*pi: #if radius is too big..
					if abs(thInit-thTemp)>2*pi:
						plant=False #we will not plant this time.
						#move to make it easier for the other head:
						if self.otherDevice is not None and self.lastPos==self.pos and self.struckLastTime:						
							thIni=self.posCyl[1]-dthini
							thTemp=thIni
							"""if exceeds(self, cart([self.posCyl[0],thTemp]), self.otherDevice):
								np=cart([self.posCyl[0],thTemp])""" #old stuff... should be removed, right?
							while inside(cart([self.posCyl[0],thTemp])) and not exceeds(self, cart([self.posCyl[0],thTemp]), self.otherDevice):
								thTemp-=dthini #moves in order to make more space
							if thTemp==thIni: #it wasnt inside or exceeded
								commands.extend(self.releaseDriver()) #releases driver, if he is used
								if exceeds(self, cart([self.posCyl[0],thTemp]), self.otherDevice):
									#we are struck! Wait for other device to move.
									self.m.stopControl() #we could have reached the end here.
									commands.append((waitevent, self, self.otherDevice.moveEvent))
								else: #not inside, we have reached the end of the half circle
									self.debugPrint("end of pattern reached, passivates %s device"%self.mountPoint)
									self.noMoreSpots=True
									self.m.stopControl() #we could have reached the end here.
									commands.append((passivate, self))
							else:
								moveTo=cart([self.posCyl[0],thTemp+dthini])
								traveltime=self.setPos(moveTo)
								self.debugPrint('clears for other head')
								commands=self.cmnd(commands, traveltime,auto=auto['clearForOtherHead'])
						if plant:
							commands=self.cmnd(commands, t['searchTime'],auto=auto['micrositeSelection'])
							self.m.timeConsumption['searchTime']+=t['searchTime']
						return (commands,plant)
					moveTo=cart([rTemp,thTemp],opt)
		travelTime=self.setPos(moveTo)
		self.debugPrint('traveltime: %f'%travelTime)
		if plant: #this timeconsumption is only for succesfull...
			commands=self.cmnd(commands, t['searchTime'],auto=auto['micrositeSelection'])
			self.m.timeConsumption['searchTime']+=t['searchTime']		
		commands=self.cmnd(commands, travelTime,auto=auto['moveToMicro'])
		return (commands,plant)
	def getNextOptimal(self):
		"""Get the next optimal position
		ideal positions cannot always be reached. if the sweep is angularly further than the ideal, choose next ideal. If we cannot reach the ideal position, 
"""
		nodes=self.optNodes
		exceeds=self.m.exceedsAngleLim
		if self.optNode is len(nodes)-1: #last node
			self.noMoreSpots=True
			return self.pos
		elif len(nodes) is 0 or (self.otherDevice is not None and exceeds(self,nodes[self.optNode+1],self.otherDevice)):
			return self.pos #could not go to next ideal, other arm is blocking.
		else:
			#get the next optimal in list and iterate until it is "forward" angularly.
			self.optNode+=1
			if self.m.type[0:2] is '2a':
				while self.m.getCylindrical(nodes[self.optNode])[1] > self.posCyl[1] and self.optNode<len(nodes)-1 and not exceeds(self,nodes[self.optNode+1],self.otherDevice): 
					self.optNode+=1
		return nodes[self.optNode]
	def plant(self):
		"""plants at the current position, may run into problems...
		the plantheads are fixed in the same direction as the crane, thus
		we define a cartesian coordinate system with origin in the center of the head
		the coordinates of the boulders can be translated into this pretty simply.
		if more than half of the boulder is inside the mounding area, it goes with it"""
		tic=time.clock()
		commands=[]
		t=self.m.times
		auto=self.m.automatic
		pHeads=self.plantHeads
		#gather information about the soil at site
		for pH in pHeads:
			pH.reset()
			moundBould=[]
			orig=pH.getPos()#middle of plantinghead
			boul=self.G.terrain.GetBoulders(orig, R=pH.radius)
			roots=self.G.terrain.GetRoots(orig,R=pH.radius)
			direct=self.m.direction-pi/2.+self.posCyl[1] #same dir as from machine to point
			sumA=0
			immobile=0.008 #as of 29th January 2012 this is the number to use.
			dibbleDisturb=0.001
			self.sim.stats['mound attempts']+=1
			#the old one: immobile=sqrt((0.20**2)/pi) #20cm rectangle side <=> tihs radius for sphere
			for r in roots: #determine if a root is hit in the critical area.
				if pH.rootCollide(r): #root is within area..
					print "striked a root.."
					if pi/4.<abs(r.direction-direct)<3*pi/4.: #abort
						pH.abort=True
						pH.done=True
					else: #remound
						print "remounds" #later..
						pH.strikedImmobile=True
					self.cmnd(commands, t['haltTime'],auto['haltMound'])
					for head in pHeads: head.timeConsumption['halting']+=t['haltTime']
			if not (pH.abort or pH.strikedImmobile):
				for b in boul:
					#get local xy-coordinates
					cylPos=self.m.getCylindrical(b.pos,origin=orig, direction=direct)
					#twoDdist=self.m.getCartesian(cylPos,origin=orig, direction=direct, local=True)#not really optimal, could be improved
					twoDdist=self.m.getCartesian(cylPos, origin=orig, direction=direct, local=True)#not really optimal, could be improved
					#if (b.z+b.radius)**2+(bpos[1])**2<pH.depth**2 and abs(bpos[0])<pH.width/2. and abs(bpos[1])<pH.length/2.: #xyz agrees old cylinder
					if (b.radius+pH.depth)**2 < b.z**2+twoDdist[1]**2 and collide(pH, b, o1pos=orig): #the first check is for the cylinder, through pythagoras with 2D[1] since cylinder and not sphere
 						#old one: abs(bpos[0])<pH.width/2. and abs(bpos[1])<pH.length/2.:
						moundBould.append(b)
						sumA+=b.area
						if b.volume>immobile:
							pH.strikedImmobile=True
							self.sim.stats['immobile boulder struck']+=1
							self.sim.stats['immobile vol sum']+=b.volume
							print "ABORTS"
							pH.abort=True
							pH.done=True
							commands=self.cmnd(commands, t['haltTime'],auto['haltMound'])
							for head in pHeads:
								head.timeConsumption['halting']+=t['haltTime'] #that's for both, if 2h
							break
						if b.radius>pH.biggestBlock:
							pH.biggestBlock=b.radius*2
				pH.moundSumA=sumA		
			pH.moundObst=moundBould
			h=Hole(orig,radius=pH.length/2.,terrain=self.G.terrain,z=pH.depth, nodes=pH.getNodes(orig) , isSpherical=False)
		commands=self.cmnd(commands, t['moundAndHeapTime'],auto['mound'])
		for pH in pHeads:
			pH.timeConsumption['mounding']+=t['moundAndHeapTime']
		self.plantSignals=0
		self.pHeadsUsed=0
		ev=[]
		for pH in pHeads:
			if not pH.abort: 
				self.pHeadsUsed+=1
				pH.cause="plant"
				pH.debugPrint("instructs pH to plant %s")
				ev.append(pH)
		if self.pHeadsUsed>0:
			commands.append(("interrupt", ev)) #will later be recognized in run and self.interupt(pH) will be invoked. 
			commands.append((waituntil, self, self.plantingFinished)) #waits for one or both events.
		PlantingDevice.timesProf[1]+=time.clock()-tic
		return commands
	def plantingFinished(self):
		if self.plantSignals==self.pHeadsUsed:
			return True
		else:
			return False
	def getNodes(self, pos=None):
		"""returns the nodes of the device."""
		if pos==None: pos=self.pos
		W=self.plantAreaW
		L=self.plantAreaL
		cart=self.m.getCartesian
		pC=self.m.getCylindrical(pos)
		l=self.plantHeads[0].length
		direction=self.m.direction-pi/2.+pC[1]
		a=cart([W/2., l/2.],origin=pos, direction=direction, local=False, fromLocalCart=True)
		b=cart([-W/2., l/2.],origin=pos, direction=direction, local=False, fromLocalCart=True)
		c=cart([-W/2., -(L-l/2.)],origin=pos, direction=direction, local=False, fromLocalCart=True)
		d=cart([W/2., -(L-l/2.)],origin=pos, direction=direction, local=False, fromLocalCart=True)
		return [a,b,c,d]
	def setPos(self,pos,c='cartesian'):
		"""Changes position for planting device, and records monitors associated with this."""
		#self.velocities={'angMachine':11.8*2*pi/60., 'angCranes': 15*pi/180, 'radial': 1} #radians/sec and m/s
		traveltime=0
		if pos != self.pos:
			self.lastPos=self.pos
			if c=='cartesian':
				self.pSpots.append(pos)
				self.pos=pos
				self.posCyl=self.m.getCylindrical(pos)
			elif c=='cylindrical':
				self.posCyl=pos
				self.pos=self.m.getCartesian(pos)
				self.pSpots.append(self.pos)
			else:
				raise Exception("ERROR: setPos only accepts cartesian and cylindrical coordinates %s"%c)
			if self.otherDevice is not None:
				#get angles in radians:
				if self.mountPoint=='left':
					[betaPr, alphaPr, thPr, r1Pr, r2Pr]=self.m.getAngles(self.pos, self.otherDevice.pos, optimize=True)
					[beta, alpha, th, r1,r2]=self.m.getAngles(self.lastPos, self.otherDevice.pos, optimize=True)
				else:
					[alpha, beta, th, r1, r2]=self.m.getAngles(self.otherDevice.pos,self.lastPos,  optimize=True)
					[alphaPr, betaPr, thPr, r1Pr, r2Pr]=self.m.getAngles(self.otherDevice.pos,self.pos, optimize=True)
				self.m.leftAngleMoni.observe(betaPr*360./(2*pi), self.sim.now())
				self.m.rightAngleMoni.observe(alphaPr*360./(2*pi), self.sim.now())
				traveltime+=self.m.timeConstants['maincrane']+abs(thPr-th)/float(self.m.velocities['angMachine'])
				#so, maincrane has moved.. time for the smaller cranes. Move them one by one.
				for arg in [(abs(alpha-alphaPr), abs(r1-r1Pr)), (abs(beta-betaPr), abs(r2-r2Pr))]:
					time=self.m.timeConstants['subcrane']+max(arg[0]/float(self.m.velocities['angCranes']), arg[1]/float(self.m.velocities['radial']))
					traveltime+=time
			else: #1a
				oldCyl=self.m.getCylindrical(self.lastPos)
				dTh=abs(oldCyl[1]-self.posCyl[1])
				dr=abs(oldCyl[0]-self.posCyl[0])
				traveltime+=self.m.timeConstants['maincrane']+max(dTh/self.m.velocities['angMachine'], dr/self.m.velocities['radial'])
			self.lastPos=self.pos #the way it should be..
			self.moveEvent.signal() #tell the other head that a movement has occured.
		self.timeConsumption['crane movement']+=traveltime
		return traveltime
	def cmndWithDriver(self, commands, time):
		"""overrides superclass method, adds priority which is important for device-head work transitions"""
		if self.usesDriver: #don't need to reserve driver..
			commands.append((hold, self, time))
			return commands
		else:
			prio=1
			for pH in self.plantHeads:
				if pH.usesDriver: #set a high priority.
					prio = 2
			commands.extend([(request, self, self.driver, prio)])
			self.usesDriver=True #this means that a reservation from the driver has been sent, not that he currently has the attention here.
			if prio==1: #we are "taking the driver" from the other device, not from our own heads
				switchTime=self.m.times['switchFocus']
				if self.driver.isIdle(): #check for how long he's been idle
					switchTime-=self.driver.idleTime()
					if switchTime<0: switchTime=0
				commands.extend([(hold, self, switchTime)]) #add time to switch focus
				self.m.timeConsumption['switchFocus']+=switchTime
			commands.extend([(hold, self, time)])
		return commands
	def getPHCoord(self, pos=None, c='cartesian'):
		if not pos:
			pos=self.pos
			cyl=self.posCyl			
		elif c=='cartesian' or c=='Cartesian':
			cyl=self.m.getCylindrical(pos)
		elif c=='cylindrical' or c=='Cylindrical':
			cyl=pos
			pos=self.m.getCartesian(pos)
		else:
			raise Exception("error, getPHCoord did not recognize %s"%c)
		if len(self.plantHeads)==1: return (pos,) #bracke
		direction=self.m.direction-pi/2.+cyl[1]
		c=self.m.getCartesian
		w=self.plantSepDist/2.
		#p1 on the left, p2 on the right
		p1=c([-w, 0], origin=pos, direction=direction, fromLocalCart=True)
		p2=c([w, 0], origin=pos, direction=direction, fromLocalCart=True)
		return p1, p2
	def getPlantingCoord(self,pos=None,c='cartesian'):
		if not pos:
			cyl=copy.copy(self.posCyl)
		elif c=='cartesian' or c=='Cartesian':
			cyl=self.m.getCylindrical(pos)
		elif c=='cylindrical' or c=='Cylindrical':
			cyl=copy.copy(pos)
		else:
			raise Exception("error, getPlantingCoord did not recognize %s"%c)
		cyl[0]-=self.plantHeads[0].length
		return self.getPHCoord(cyl, c='cylindrical')
	def getPhNodes(self,pH, pos=None, rootA=False):
		"""returns the nodes of the planthead pH. Needs to be here for directions and so on.
		rootA means that the area where the roots affects the mounding are asked for."""
		if pos==None: 
			pos=self.pos
			[r,th]=self.posCyl
			if len(self.plantHeads)==1:
				orig=self.getPHCoord(pos)[0]
			else:
				[p1, p2]=self.getPHCoord(pos)
				if pH.leftRight=='left': orig=p1
				else: orig=p2
		else:
			#first, determine the direction of the CRANE
			orig=pos
			posCyl=self.m.getCylindrical(pos)
			#this may seem like a sign-error, but remember that pos is pH-pos. Need th for self, not pH
			if len(self.plantHeads)==1: #bracke
				th=posCyl[1]
			elif pH.leftRight=='left': th=posCyl[1]-asin(self.plantSepDist/2./posCyl[0])
			else: th=posCyl[1]+asin(self.plantSepDist/2./posCyl[0])
		direction=self.m.direction+th-pi/2.
		cart=self.m.getCartesian
		w=pH.width
		l=pH.length
		if rootA:
			w=w*0.75
			l=l*0.75
		c1=cart([w/2., l/2.],origin=orig, direction=direction, local=False, fromLocalCart=True)
		c2=cart([-w/2., l/2.],origin=orig, direction=direction, local=False, fromLocalCart=True)
		c3=cart([-w/2., -l/2.],origin=orig, direction=direction, local=False, fromLocalCart=True)
		c4=cart([w/2., -l/2.],origin=orig, direction=direction, local=False, fromLocalCart=True)
		return [c1,c2,c3,c4]
	def draw(self, ax):
		#plot ideal spots
		if self.mountPoint=='left':
			color='y'
			sign='o'
		else:
			color='r'
			sign='s'
		"""x=[]
		y=[]
		for i in self.idealSpots:
			x.append(i[0])
			y.append(i[1])
					 
			#ax.plot(i[0], i[1],color+sign)
		ax.plot(x,y, 'b')"""
		if self.G.debug:
			#plot visited spots
			x=[]
			y=[]
			for spot in self.pSpots:
				x.append(spot[0])
				y.append(spot[1])
				ax.plot(spot[0], spot[1], color+'o')
			if self.mountPoint=='right': ax.plot(x,y, color+'--')
			else: ax.plot(x,y, color+'--')
		nodes=self.getNodes()
		verts=[]
		codes=[]
		verts.append(nodes[0])
		codes.append(Path.MOVETO)
		for corner in nodes[1:]:
			verts.append(corner)
			codes.append(Path.LINETO)
		verts.append((5,5)) #ignored
		codes.append(Path.CLOSEPOLY)
		path = Path(verts, codes)
		patch = patches.PathPatch(path, facecolor='y', lw=2, alpha=95)
		ax.add_patch(patch)
