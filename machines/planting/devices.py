from SimPy.Simulation  import *
import machines
from machines.basics import Machine, UsesDriver, Operator
from heads import PlantHead, Mplanter, Bracke, MultiHead
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
		#crane starts in its minimum position to the left of the driver.
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
			self.plantSepDist=2.01 #existing machine, constant parameter
			for no in [0,1]:
				#h=Mplanter(self.name+"_"+str(no)+"_head", self.sim, PD=self,leftRight=lr)
				h=Mplanter(self.name+"_"+str(no)+"_head", self.sim, PD=self,number=no)
				self.sim.activate(h,h.run())
				self.plantHeads.append(h)
			self.plantAreaW=self.plantSepDist+h.width
		elif self.m.headType=='MultiHead':
			self.plantSepDist=self.G.simParam['dibbleDist'] #1m by default
			if '3h' in self.m.type:
				no=3
			elif '4h' in self.m.type:
				no=4
			else:
				raise Exception('3 or 4 heads are supported, not more or less, %s'%str(self.m.type))
			for i in range(no):
				h=MultiHead(self.name+"_"+str(i+1)+"_head", self.sim, PD=self,number=i)
				self.sim.activate(h,h.run())
				self.plantHeads.append(h)
			self.plantAreaW=(no-1)*self.plantSepDist+h.width #2*h.width/2..both sides..
		elif self.m.headType=='Bracke':
			h=Bracke(self.name+"_"+"brackeHead", self.sim, PD=self)
			self.sim.activate(h, h.run())
			self.plantHeads.append(h)
			self.plantAreaW=h.width
		else:
			raise Exception('Headtype not supported: %s'%str(self.m.headType))
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
		"""
		sets the pattern for optimal positions.
		"""
		self.optNodes=[]
		self.optNode=-1
		if self.m.headType=='Bracke':
			bracke=True #one head per decice
		else:
			bracke=False
		if '2a' in self.m.type:
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
		else:
			assert len(self.m.pDevs)==0 or len(self.m.pDevs)==1 and self.m.pDevs[0]==self
			w1 = self.plantAreaW/2.
			w2 = self.plantAreaL/2.
			if bracke:
				spaceMin=self.m.plantMinDist
			else:
				spaceMin=self.plantAreaW-self.plantHeads[0].width+self.m.plantMinDist #minimum spacing for angular movements.
			n=ceil(self.m.nSeedlingsPWArea/len(self.plantHeads)) #due to several plantHeads per device
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
			if '2a' in self.m.type:
				while self.m.getCylindrical(nodes[self.optNode])[1] > self.posCyl[1] and self.optNode<len(nodes)-1 and not exceeds(self,nodes[self.optNode+1],self.otherDevice): 
					self.optNode+=1
		return nodes[self.optNode]
	def plant(self):
		"""
		plants at the current position, may run into problems...
		the plantheads are fixed in the same direction as the crane, thus
		we define a cartesian coordinate system with origin in the center of the head
		the coordinates of the boulders can be translated into this pretty simply.
		if more than half of the boulder is inside the mounding area, it goes with it

		This is done here since it's all a common crane movement. Thus it should be at the same time for all the heads and is rather connected to the device than the heads. If something happens in a specific head that the other heads have to queue for, we simple make it something that takes time for the device and not the heads.
		"""
		tic=time.clock()
		commands=[]
		t=self.m.times
		auto=self.m.automatic
		pHeads=self.plantHeads
		#gather information about the soil at site
		deepest=0
		deepestPos=None
		for h in pHeads:
			depth=self.G.terrain.humusLayer.getDepth(h.getPos())
			assert depth>=0
			if depth>deepest:
				deepest=depth
				deepestPos=h.getPos()
		depth=deepest
		digTime=self.m.getDigTime(deepestPos)
		self.sim.stats['humus depths'].append(depth)
		if self.m.inverting: #determine the time. Dependent on digTime
			if self.m.invertingMethod=='KO':
				invertTime=self.G.simParam['tCWhenInvKO']
			elif self.m.invertingMethod=='Excavator':
				invertTime=self.G.simParam['tInvExcavator']-digTime
			else:
				raise Exception('cannot identify inverting method %s'%self.m.invertingMethod)
		for pH in pHeads:
			pH.reset()
			moundBould=[]
			orig=pH.getPos()#middle of plantinghead
			boul=self.G.terrain.GetBoulders(orig, R=pH.radius)
			roots=self.G.terrain.GetRoots(orig,R=pH.radius)
			direct=self.m.direction-pi/2.+self.posCyl[1] #same dir as from machine to point
			sumA=0
			immobile=self.G.simParam['critStoneSize']
			dibbleDisturb=0.001
			self.m.stopControl()
			self.sim.stats['mound attempts']+=1
			for r in roots: #determine if a root is hit in the critical area.
				if pH.rootCollide(r): #root is within area..
					print "striked a root.."
					angle=abs(r.direction-direct)
					ray1=[orig,fun.getCartesian([0,1],fromLocalCart=True, origin=orig, direction=r.direction)]
					ray2=[orig,fun.getCartesian([0,1],fromLocalCart=True, origin=orig, direction=direct)]
					angle=fun.getAngle(ray1, ray2) #angle between root and planting head
					pH.strikedImmobile=True
					self.cmnd(commands, t['haltTime'],auto['haltMound'])
					for head in pHeads: head.timeConsumption['halting']+=t['haltTime']
					if self.G.simParam['noRemound'] or angle>self.m.rootDegreesOK:
						self.debugPrint('pos: %s collided with root. angle was too much %s'%(str(orig), str(angle*180.0/pi)))
						pH.abort=True
						pH.done=True
					else: #remound
						print "remounds"
						self.cmnd(commands, t['haltTime'],auto['haltMound'])
						timeTmp=digTime+t['heapTime']
						self.cmnd(commands, timeTmp, auto['mound'])
						for pH in pHeads:
							pH.timeConsumption['halting']+=t['haltTime'] #that's for both, if 2h
							pH.remounded=True
							pH.timeConsumption['mounding']+=timeTmp
						

					
			if not (pH.abort or pH.strikedImmobile):
				for b in boul:
					#check if we are inside the scoop. It's the middle of the stone that matters
					#get local xy-coordinates
					cylPos=self.m.getCylindrical(b.pos,origin=orig, direction=direct)
					twoDdist=self.m.getCartesian(cylPos, origin=orig, direction=direct, local=True)#not really optimal, could be improved
					inside=False #just to skip a really long if-statement
					if self.G.simParam['rectangular']:
						if b.radius+b.z>-pH.depth and collide(pH, b, o1pos=orig):
							inside=True
					elif b.z**2+twoDdist[1]**2<(b.radius+pH.depth)**2 and collide(pH, b, o1pos=orig): #the first check is for the cylinder, through pythagoras with 2D[1] since cylinder and not sphere
						inside=True
					if inside: 
 						#old one: abs(bpos[0])<pH.width/2. and abs(bpos[1])<pH.length/2.:
						moundBould.append(b)
						sumA+=b.area
						localPos=-twoDdist[1], b.z #2D position with z as y-axis
						#now, look how much it occuppies vertically.
						twoDdist=self.m.getCartesian(cylPos, origin=orig, direction=direct, local=True)#not really optimal, could be improved
						if self.G.simParam['rectangular']:
							nodes=[(-pH.length*0.5,0), (-pH.length*0.5, -pH.depth), (pH.length*0.5, -pH.depth), (pH.length*0.5, 0)]
							last=None
							points=[]
							for node in nodes:#loop over the rectangle edges.
								if last:
									ray=(last,node)
									tmp=col.intersectRaySphere(np.array(ray),b.radius,localPos, additionalInfo=True)
									if type(tmp)!=bool:
										for point in tmp[1:]:
											points.append(list(point))
								last=node
							assert len(points)!=1 #would be tangent but..
							upper=(-twoDdist[1], b.z+b.radius)
							lower=(-twoDdist[1], b.z-b.radius)
							if not col.pointInPolygon(upper, nodes):
								if len(points)==0: #it passed through the easy check above...
									upper=-pH.depth
									moundBould.remove(b)
									sumA-=b.area
								else:
									upper=max([p[1] for p in points])
							else:
								upper=upper[1]
							if not col.pointInPolygon(lower, nodes):
								if len(points)==0:
									lower=-pH.depth
								else:
									lower=min([p[1] for p in points])
							else:
								lower=lower[1]
						else:
							r=b.radius
							#look how much of the stone that is within the scoop.
		
							points=col.circlesIntersectPoints((0,0), localPos, pH.depth, b.radius)
							assert points != False # we know that these circles collide.
							if points== True: #all of the stone inside or huge stone
								upper=b.z+b.radius
								lower=b.z-b.radius
							else:
								upper=max(points[0][1], points[1][1])
								if col.pointInCircle((-twoDdist[1], b.z+b.radius), (0,0), pH.depth):
									assert b.z+b.radius>=upper
									upper=b.z+b.radius
								lower=min(points[0][1], points[1][1])
								if col.pointInCircle((-twoDdist[1], b.z-b.radius), (0,0), pH.depth):
									assert b.z-b.radius<=lower
									lower=b.z-b.radius
						hInside=upper-lower
						assert hInside>=0
						ratio=hInside/float(pH.depth)
						pH.strikedImmobile=True
						self.sim.stats['immobile boulder struck']+=1
						self.sim.stats['immobile vol sum']+=b.volume
						if ratio>self.m.immobilePercent:
							self.debugPrint("ABORTS %s percent is vertically occupided by an imobile boulder"%str(ratio))
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
			h=Hole(orig,terrain=self.G.terrain,z=pH.depth, nodes=pH.getNodes(orig) , isSpherical=False)
		#time to mound and heap. With the Excavator inverting method, we don't take time for heaping now.
		if not self.m.inverting:
			timeTmp=digTime+t['heapTime']
			commands=self.cmnd(commands, timeTmp,auto['mound'])
		elif self.m.inverting and self.m.invertingMethod=='KO': #heap first..
			timeTmp=digTime+t['heapTime']
			commands=self.cmnd(commands, timeTmp,auto['mound'])
		elif self.m.inverting and self.m.invertingMethod=='Excavator': #don't heap..
			timeTmp=digTime
			commands=self.cmnd(commands, timeTmp,auto['mound'])
		else:
			raise Exception('Logical error. If we are inverting, we need to use methods KO or Excavator, not %s'%self.invertingMethod)
		for pH in pHeads:
			pH.timeConsumption['mounding']+=timeTmp
		#mounding failures
		for h in self.plantHeads:
			if random.uniform(0,1)<self.m.moundingFailureProb and not h.remounded: #failure..
					
				if self.G.simParam['noRemound']:
					h.debugPrint('failed mounding')
					h.abort=True
				else:
					h.debugPrint('Failed mounding.. the other heads have to wait')
					commands=self.cmnd(commands, digTime+t['heapTime'],auto['mound'])
					for pH in self.plantHeads:
						self.sim.stats['remound attempts']+=1
						pH.timeConsumption['mounding']+=digTime+t['heapTime']
						pH.remounded=True
		#it's time to invert
		if self.m.inverting:
			commands=self.cmnd([], invertTime, auto=False)
			reinverted=False
			reinvertTime=digTime+t['heapTime'] #same for KO and Excv
			for h in self.plantHeads:
				if pH.abort: continue
				self.sim.stats['inverting attempts']+=1
				h.timeConsumption['inverting']+=invertTime
				if random.uniform(0,1)<self.m.invertFailureProb: #failure..
					self.debugPrint('reinverts')
					if self.G.simParam['noRemound']:
						h.debugPrint('failed inverting')
						h.abort=True
					elif not reinverted:
						reinverted=True
						h.debugPrint('Failed mounding.. the other heads have to wait')
						commands=self.cmnd(commands,reinvertTime,auto['mound'])
						for pH in self.plantHeads:
							self.sim.stats['reinverting attempts']+=1
							h.timeConsumption['inverting']+=reinvertTime
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
		#from left to right
		positions=[]
		x=-(self.plantAreaW*0.5-self.plantHeads[0].width*0.5)
		for i in range(len(self.plantHeads)):
			positions.append(c([x, 0], origin=pos, direction=direction, fromLocalCart=True))
			x+=self.plantSepDist
		return positions
	
	def getPlantingCoord(self,pos=None,c='cartesian'):
		if not pos:
			cyl=copy.copy(self.posCyl)
		elif c=='cartesian' or c=='Cartesian':
			cyl=self.m.getCylindrical(pos)
		elif c=='cylindrical' or c=='Cylindrical':
			cyl=copy.copy(pos)
		else:
			raise Exception("error, getPlantingCoord did not recognize %s"%c)
		if not self.m.inverting: cyl[0]-=self.plantHeads[0].length #a pile behind the hole
		return self.getPHCoord(cyl, c='cylindrical')
	
	def getPhNodes(self,pH, pos=None, rootA=False):
		"""
		returns the nodes of the planthead pH. Needs to be here for directions and so on.
		rootA means that the area where the roots affects the mounding are asked for.
		"""
		if pos==None: 
			pos=self.pos
			[r,th]=self.posCyl
			if len(self.plantHeads)==1:
				orig=self.getPHCoord(pos)[0]
			else:
				positions=self.getPHCoord(pos)
				orig=positions[pH.number]
		else:
			#first, determine the direction of the CRANE
			orig=pos
			posCyl=self.m.getCylindrical(pos)
			#this may seem like a sign-error, but remember that pos is pH-pos. Need th for self, not pH
			if len(self.plantHeads)==1: #bracke
				th=posCyl[1]
			else:
				x=-(self.plantAreaW*0.5-pH.width*0.5)
				x+=pH.number*self.plantSepDist #this is the upper part of the triangle.
				th=posCyl[1]+asin(x/posCyl[0])
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
		x=[]
		y=[]
		"""for i in self.idealSpots:
			x.append(i[0])
			y.append(i[1])
					 
			#ax.plot(i[0], i[1],color+sign)
		ax.plot(x,y, '-bo')"""
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
