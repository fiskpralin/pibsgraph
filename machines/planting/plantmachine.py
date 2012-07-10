from SimPy.Simulation  import *
import machines
from machines.basics import Machine, UsesDriver, Operator
from machines.planting.devices import PlantingDevice
from heads import PlantHead, Mplanter, Bracke
import terrain
from terrain.obstacle import Obstacle
from terrain.tree import Tree
from terrain.hole import Hole
from terrain.seedling import Seedling
from terrain.stump import Stump
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

	The below machine types are allowed. If you plan to add a new one, consider that the program does tests on the type, e.g. "if '2a' in type" to determine if the machine has two arms. If e.g. '2angles' would be the name of the new type, we would get a problem.
	"""
	allowedTypes= ['1h', '2h','1a1h','1a2h','2a1h', '2a2h', '1a2hObAv','1a3h', '3h','1a3hObAv','1a4h','4h','1a4hObAv','1a1hMag','1a2hMag']
	def __init__(self, name, sim, G, mtype='2h', craneLim=None):
		#if not mtype in self.allowedTypes or 'ObAv' in mtype:
		#	raise Exception('Machine type %s no allowed'%str(mtype))
		if not craneLim: craneLim=[4.0,9.0]
		Machine.__init__(self, name, sim, G=G, mass=21000)
		self.driver=Operator(sim=self.sim, delay=10000) #does not go on breaks..
		self.sim.activate(self.driver, self.driver.work())
		self.type=mtype
		if '2h' in self.type:
			self.headType='Mplanter'
		elif '3h' in self.type or '4h' in self.type:
			self.headType='MultiHead' #several heads, don't know how many yet.
		else:
			self.headType='Bracke'
		self.times={'heapTime': 2,'dibbleDownTime': 1, 'relSeedlingTime': 1, 'dibbleUpTime':1, 'haltTime': 1, 'searchTime': 0, 'switchFocus':0, 'invertTime':G.simParam['tCWhenInvKO'], 'invert':None} #all in seconds, invert is determined later
		self.timeConsumption={'heapTime': 0,'moundAndHeapTime': 0, 'dibbleDownTime': 0, 'relSeedlingTime': 0, 'haltTime': 0, 'searchTime': 0, 'switchFocus':0, 'machineMovement':0} #new function for digTime, see plant method.
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
		self.stockingRate=self.G.simParam['TSR']
		self.plantMinDist=G.simParam['dibbleDist']
		self.dibbleDepth=0.1
		self.moundingFailureProb=G.simParam['moundFailureProb']=0.05
		self.inverting=G.simParam['inverting']
		if self.inverting:
			assert G.simParam['ExcavatorInverting']!=G.simParam['KOInverting'] #can't use both			
			if G.simParam['KOInverting']:
				self.invertingMethod='KO'
				self.invertFailureProb=G.simParam['invertKOFailureProb']
			else:
				self.invertingMethod='Excavator'
				self.invertFailureProb=G.simParam['invertExcFailureProb']			
		else:
			assert not G.simParam['KOInverting'] and not G.simParam['ExcavatorInverting']
		self.nSeedlingsPWArea=max(floor(self.stockingRate/10000.*self.workingArea),1)
		print "sPerWorkarea:", self.nSeedlingsPWArea, "cranemax:", self.craneMaxL, "cranemin:",self.craneMinL, "attach:", self.craneIntersect
		#self.direction=random.uniform(0,2*pi)
		self.direction=0
		
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
		if '2a' in self.type: #add another one. 1a is default
			self.times['switchFocus']=2
			self.timeConstants['machine']*=1.5 #longer time for 2a
			self.velocities['machine']*=0.75 #heavier, takes more time to move. Part of BTE model
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
		self.inPlaceEvent=SimEvent('machine has moved in place', sim=self.sim) #event fired when machine has moved
		self.calcVisibleObstInWA() #some statistics updates needed
		if self.headType=='Mplanter' or self.headType=='MultiHead':
			try:
				multiplier=self.G.simParam['tFindMuSite']
			except:
				multiplier=0.1 #default..
			self.times['searchTime']=multiplier*self.sim.stats['visible obstacles in WA'] #specifics for this head..0 otherwise
		elif self.headType=='Bracke':
			pass
		else:
			raise Exception('could not identify head type %s'%self.headType) #safety first..
		
	def run(self): #the method has to be here in order to be counted as an entity
		#get machine in place. Assumes that machine comes from last position in half-cricle pattern.
		distance=6.0 #not exact, half circle may overlap more or less than this.

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
	def getDigTime(self, pos):
		"""
		returns the time to dig at pos
		"""
		return 3+0.1*(self.G.terrain.humusLayer.getDepth(pos)-0.1)
	def stopControl(self):
		"""
		Checks if simulations should be stopped
		"""
		reason=None
		if self.driver.resting:
			return False #wait until he's not at rest, should not result in an infinite loop.
		elif len(self.treesPlanted)>=floor(self.stockingRate/10000.0*self.workingArea):
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
		print self.sim.stats['mound attempts'],len( self.treesPlanted)
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
	def calcVisibleObstInWA(self):
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
		potentialObst=[o for o in self.G.terrain.GetVisibleObstacles(self.pos, self.craneMaxL) if o.visible]
		obstNo=0
		sdiam=0
		for o in potentialObst:
			if collide(o,pol):
				obstNo+=1
				if isinstance(o, Stump):
					sdiam+=o.dbh
		self.sim.stats['visible obstacles in WA']=obstNo
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
			

