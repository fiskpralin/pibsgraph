from SimPy.Simulation  import *

from machines.basics import UsesDriver, Machine, Operator
from thHeads import ThinningCraneHead, BCHead, ConventionalHead, ConventionalHeadAcc
from thRoad import ThinningRoad
from terrain.obstacle import Obstacle
from terrain.tree import Tree
import collision as col
import functions as fun
from math import *
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import copy
import time
"""This module stores all the machines."""


class ThinningMachine(Machine, UsesDriver):
	"""
	A thinningmachine that thins a 40m long road
	
	All measurements are taken from the komatsu 901.4 harvester with 620/55 x 30,5 wheels on the back and 650/45 x 22,5 wheels in the front.
	"""
	def __init__(self, name, sim, G, head='BC', nCranes=2, startPos=[12.5, -4]):
		print "head:", head, "cranes:", nCranes
		self.driver=Operator(sim) #add driver
		sim.activate(self.driver, self.driver.work())
		Machine.__init__(self, name, sim, G=G, driver=self.driver, mass=21000)
		self.velocities={'machine': 1, 'crane angular':0.35, 'crane radial': 2.5, 'fell':0.08} #radians/sec,m/s and m2/s
		self.color='#CD0000'
		self.moveEvent=SimEvent(name='machine moves', sim=self.sim) #signals BEFORE movement
		self.movedEvent=SimEvent(name='machine moves', sim=self.sim) #signals AFTER movement
		if self.G.automatic != 'undefined':
			self.automatic=self.G.automatic
		else: #give default automation
			if head=='BC':
				self.automatic={'move': False, 'moveArmIn': True, 'moveArmOut': False, 'dumpTrees': False, 'switchFocus': False, 'chop':True} #default, override before activating machine
			elif head=='conv':
				self.automatic={'move': False, 'moveArmIn': False, 'moveArmOut': False, 'dumpTrees': False, 'switchFocus': False, 'chop':False} #default, override before activating machine
			elif head=='convAcc':
				self.automatic={'move': False,'moveArmIn': False, 'moveArmOut': False, 'dumpTrees': False, 'switchFocus': False, 'chop': False}
			else: raise Exception('head type could not be identified', head)
			
		self.times={'crane const':1.5, 'chop const':3, 'move const':5, 'dumpTrees': 10, 'switchFocus': 3} #same as above, change before activating. may be changed from cranHead constructor
		self.craneMaxL=11
		self.craneMinL=3
		self.length=6.939
		self.width=2.720
		self.pos=startPos
		self.trees=[]
		self.mainRoadTrees=[]
		self.corridorTrees=[]
		self.heads={} #dictionary with keys 'left', 'right'
		if head=='BC':
			for i in range(nCranes):
				h=BCHead(sim=self.sim, driver=self.driver, machine=self) #adds to above list.
		elif head=='conv':
			for i in range(nCranes):
				h=ConventionalHead(sim=self.sim, driver=self.driver, machine=self) #adds to above list.
		elif head=='convAcc':
			h=ConventionalHeadAcc(sim=self.sim, driver=self.driver, machine=self) #adds to above list.
		else:
			raise Exception('ThinningMachine did not recognize head %s'%str(head))
		for h in self.heads.values():
			self.sim.activate(h, h.run())
		self.treeMoni=Monitor(name='trees harvested')
		self.treeMoni.observe(len(self.trees), self.sim.now())
		self.roadList=[] #simply a list of roads.
		self.roads={} #a dictionary, more sophisticated structure with simplifies finding the road for a spec. pos.
		self.setUpCorridors()
	def run(self):
		"""
		PEM of thinning machine
		"""
		#first, take the trees in the main road:
		while True:
			r=self.roads['main']
			self.heads['left'].road=r #only one head in the two armed case because of balance. left is default 1a.
			print "Mainroad assigned"
			yield waituntil, self, self.headsDoneAtSite
			#check if road is clear until the next position.
			p=self.getRoadClearPos()
			while p != self.getNextPos(): #if not clear, move some more and clear the road.
				for c in self.setPos(p, cmnd=True): yield c
				for a in self.releaseDriver(): yield a 
				self.movedEvent.signal()
				self.heads['left'].road=r
				yield waituntil, self, self.headsDoneAtSite
				p=self.getRoadClearPos()
			#p is self.getNextPos()
			for c in self.setPos(p, cmnd=True): yield c
			if self.getNextPos()==self.pos:
				self.sim.stopSimulation()
				yield hold, self, 0.001 #give it time to stop..
			r=self.roads[self.pos[1]] #current roads.. should be like 10 of them..
			for h in self.heads.values():
				if len(r[h.side])>0: h.road=r[h.side][0]
			for c in self.releaseDriver(): yield c
			yield waituntil, self, self.headsDoneAtSite
	def headsDoneAtSite(self):
		"""are the heads done?"""
		for h in self.heads.values():
			if h.road: return False #if road is assigned, head is still working.
		return True
	
   	def setUpCorridors(self):
		"""
		Sets up 
		Note:
		this is a very inefficient way of doing it, using an iterative method.
		However, it is sufficient for current use and can be improved if needed.
		"""
		#main road
		print "sets up roads"
		startX=self.pos[0]
		tic=time.clock()
		W=4.0 #standard..
		cart=self.getCartesian
		h1=self.heads['left']
		origin=[startX, 0]
		c1=cart([-W/2., 0],origin=origin, fromLocalCart=True)
		c2=cart([-W/2., 40],origin=origin, fromLocalCart=True)
		c3=cart([W/2., 40],origin=origin, fromLocalCart=True)
		c4=cart([W/2., 0],origin=origin, fromLocalCart=True)
		mainRoad=ThinningRoad([startX, 20], [c1,c2,c3,c4], G=self.G, direction=-pi/2., machine=self, main=True)
		mainRoad.add()
		self.roads['main']=mainRoad #each new point will get a new instance here.
	
		#the key for each corridor set is first the y-pos of the vehicle, then the side, e.g. corridor[5.5]['left'] gives three corridors
		P=[startX,5.5] #first stop by default
		self.positions=[self.pos, P]
		w=self.heads['left'].corridorWidth #meters
		L=sqrt(self.craneMaxL**2-W**2)
		nPerSide=self.heads['left'].corrPerSide
		sigma=pi/nPerSide/3. #allowed deviation from optimal angle.
		points=10 #integer no of points is the double
		angMin=pi/4. #no "vertical" corridors
		rad=sqrt(w**2/4+L**2/4) #radius of corridor.. saves time
		while True:
			self.roads[P[1]]={}
			for side in ['left', 'right']:
				cTemp=[]
				mod=1
				if side is 'right': mod=-1 #distinguishes between the two sides.
				for ang in [mod*(angMin+(pi-2*angMin)/(nPerSide-1)*(i)) for i in range(nPerSide)]: #e.g. [pi/4., pi/2., 2/4.*pi]:
					most=0
					best=None
					for sig in [x * sigma/(points*2) for x in range(-points, points)]:
						#define the coordinates.
						direction=pi/2.+ang+sig
						sp=cart([0, abs(W/2./sin(direction-pi/2.))],origin=P, direction=direction, fromLocalCart=True) #startPos
						w2=w/cos(-direction) #to compensate for the angular effect on the side.
						if side is 'left':
							w2=-w2
							c1=[sp[0], sp[1]-w2/2.]
							c4=[sp[0], sp[1]+w2/2.]
						else:
							c4=[sp[0], sp[1]-w2/2.]
							c1=[sp[0], sp[1]+w2/2.]
						c2=cart([-w/2., L],origin=P, direction=direction, fromLocalCart=True)
						c3=cart([w/2., L],origin=P, direction=direction, fromLocalCart=True)
						pos=cart([0, L/2.],origin=P, direction=direction, fromLocalCart=True)
						c=ThinningRoad(pos, [c1,c2,c3,c4], G=self.G, direction=direction, machine=self, radius=rad) #radius not completely correct.. but speeds things up.
						c.startPoint=sp
						if len(c.trees)==0: c=None #no trees in c, no meaning to have..
						else:
							for t in c.trees:
								if not h1.treeChopable(t):
									c=None #don't use this road.. bad trees in the way.
									break
							if c and c.harvestTrees>most:
								best=c
								most=c.harvestTrees
					if best:
						best.add()
						cTemp.append(best)
						#print best.direction*180/pi
				self.roads[P[1]][side]=cTemp
			P=copy.copy(P) #there are references to the old P.
			P[1]=P[1]+cos(angMin)*self.craneMaxL
			if P[1]>self.G.terrain.ylim[1]: break
			self.positions.append(P)
		self.positions.append([self.positions[0][0], self.G.terrain.ylim[1]]) #the last spot.
		print "time to set up roads: %f"%(time.clock()-tic)
	def getNextPos(self):
		"""
		returns the next stop
		"""
		i=0
		for p in self.positions:
			if p[1]>self.pos[1]: return p
			if len(self.positions)<i+2: return self.pos
			i+=1
		raise Exception('getNextPos is not expected to come this far.')
	def getTreeDumpSpot(self, side):
		"""
		returns the position to dump the trees
		"""
		cart=self.getCartesian
		W=2
		L=self.length/3.
		if side=='left':
			return cart([-W,-L], fromLocalCart=True)
		elif side=='right':
			return cart([W, -L], fromLocalCart=True)
		else: raise Exception('getTreeDumpSpot does not recognize side %s'%side)
	def getNodes(self, pos=None):
		"""
		if position is not current position, it is expected that vechicle goes there from the current
		position, which gives the angle.
		"""
		direction=self.direction
		if not pos:
			pos=self.pos
		elif pos != self.pos:
			[r,direction]=self.getCartesian(pos, direction=pi/2)
		#get the nodes going!
		W=1
		L=3
		c=self.getCartesian
		nodes=[]
		nodes.append(c([W/2., L/2.], origin=pos, direction=direction, fromLocalCart=True))
		nodes.append(c([-W/2., L/2.], origin=pos, direction=direction, fromLocalCart=True))
		nodes.append(c([-W/2., -L/2.], origin=pos, direction=direction, fromLocalCart=True))
		nodes.append(c([W/2., -L/2.], origin=pos, direction=direction, fromLocalCart=True))
		return nodes
	def setPos(self, pos, cmnd=False):
		time=super(ThinningMachine,self).setPos(pos)+self.times['move const']
		for h in self.heads.values():
			h.pos=h.getStartPos() #crane are always in this pos while moving
		if cmnd: return self.cmnd([],time, self.automatic['move'])
		else: return time
	def getRoadClearPos(self):
		"""used for the mainRoad. Check if the road is clear or if we have to take a small movement first and clear it."""
		p=self.getNextPos()
		closest=None
		r=self.roads['main']
		for t in r.trees:
			if not closest or t.pos[1]<closest.pos[1]:
				closest=t
		if closest and p[1]+self.radius>=closest.pos[1]-closest.dbh:
			pnew= [p[0], closest.pos[1]-closest.dbh-self.radius]
			return pnew
		else:
			return p
	def draw(self,ax):
		"""draws the machine at current poistion and with current direction.
		 All measurements are taken from the komatsu 901.4 harvester with 620/55 x 30,5 wheels on the back and 650/45 x 22,5 wheels in the front."""
		cart=self.getCartesian
		#draw roads:
		for road in [r for r in self.roadList if (r.radius>self.G.terrain.ylim[1]/2. or r.harvestTrees==0)]+[h.road for h in self.heads.values() if h.road and h.road.harvestTrees != 0 and h.road.radius<self.G.terrain.ylim[1]/2.]:
			road.draw(ax)	#main roads have r>c+3. Current roads and visited roads
		#wheels. first the four front wheels.
		w=0.225
		r=0.650
		W=2.650
		f=1.850+0.650+0.1
		o1=cart([W/2.-w/2., f], fromLocalCart=True)
		o2=cart([-(W/2.-w/2.), f], fromLocalCart=True)
		o3=cart([-(W/2.-w/2.), 1.850-0.1-r], fromLocalCart=True)
		o4=cart([W/2.-w/2., 1.850-0.1-r], fromLocalCart=True)
		for origin in [o1,o2,o3,o4]:
			c1=cart([w/2., r], origin=origin, fromLocalCart=True)
			c2=cart([-w/2., r], origin=origin, fromLocalCart=True)
			c3=cart([-w/2., -r], origin=origin, fromLocalCart=True)
			c4=cart([w/2., -r], origin=origin, fromLocalCart=True)
			p=mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k')
			ax.add_patch(p)
		#back wheels:
		W=2.720
		w=0.305
		r=0.620
		o1=cart([W/2.-w/2.,-1.650],fromLocalCart=True)
		o2=cart([-W/2.+w/2.,-1.650],fromLocalCart=True)
		for origin in [o1,o2]:
			c1=cart([w/2., r], origin=origin, fromLocalCart=True)
			c2=cart([-w/2., r], origin=origin, fromLocalCart=True)
			c3=cart([-w/2., -r], origin=origin, fromLocalCart=True)
			c4=cart([w/2., -r], origin=origin, fromLocalCart=True)
			p=mpl.patches.Polygon(np.array([c1,c2,c3,c4]), closed=True, facecolor='k')
			ax.add_patch(p)
		#base structure
		L=6.939
		W=2.720
		a=1.850+2*0.650+0.1
		f=1.850+0.650+0.1
		c=[cart([(W-2*0.305)/2., f],fromLocalCart=True)]
		c.append(cart([-(W-2*0.305)/2., f],fromLocalCart=True))
		#fix a less wide end.
		w=(W-2*0.305)/2.*0.70 #width at the back
		#some coordinates are for the red parts.. others for the base structure
		diff=0.1 #difference between red and black parts
		r2=cart([-(W-2*0.305)/2.+diff, -1.650+r-0.05],fromLocalCart=True)
		c2=cart([-(W-2*0.305)/2., -1.650],fromLocalCart=True)
		r21=cart([-(W-2*0.305)/2.+diff, -1.650],fromLocalCart=True)
		r5=cart([(W-2*0.305)/2.-diff, -1.650+r-0.05],fromLocalCart=True)
		c5=cart([(W-2*0.305)/2., -1.650],fromLocalCart=True)
		r41=cart([(W-2*0.305)/2.-diff, -1.650],fromLocalCart=True)
		r3=cart([-w+diff, a-L+diff],fromLocalCart=True)
		c3=cart([-w, a-L],fromLocalCart=True)
		r4=cart([w-diff, a-L+diff],fromLocalCart=True)
		c4=cart([w, a-L],fromLocalCart=True)
		c.extend([c2,c3,c4,c5])
		p=mpl.patches.Polygon(np.array(c), closed=True, facecolor='k', alpha=90)
		ax.add_patch(p)
		p=mpl.patches.Polygon(np.array([r2, r21,r3,r4, r41,r5]), closed=True, facecolor=self.color)
		ax.add_patch(p)
		#draw the heads and crane:
		for h in self.heads.values():
			h.draw(ax)	
		#cabin:
		l=1.8 #pure estimation for these three variables
		w=W-2*r
		f=0.4
		smooth=1/5.*min(l,w)
		origin=cart([0,f],fromLocalCart=True)
		c2=cart([w/2., l/2.-smooth], origin=origin, fromLocalCart=True)
		c3=cart([w/2.-smooth, l/2.], origin=origin, fromLocalCart=True)
		c5=cart([-w/2.+smooth, l/2.], origin=origin, fromLocalCart=True)
		c6=cart([-w/2., l/2.-smooth], origin=origin, fromLocalCart=True)
		c8=cart([-w/2., -l/2.+smooth], origin=origin, fromLocalCart=True)
		c9=cart([-w/2.+smooth, -l/2.], origin=origin, fromLocalCart=True)
		c11=cart([w/2.-smooth, -l/2.], origin=origin, fromLocalCart=True)
		c12=cart([w/2., -l/2.+smooth], origin=origin, fromLocalCart=True)
		a=[c2,c3,c5,c6,c8,c9,c11,c12]
		ax.add_patch(mpl.patches.Polygon(np.array(a), closed=True, facecolor=self.color))

