#!/usr/bin/env python
import sys
import copy
import time
from math import *

from SimPy.Simulation  import *
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pylab
import networkx as nx
from matplotlib.patches import Circle
from matplotlib.patches import Rectangle 

import terrain
from road import Road
import machines
from machines.basics import Machine
from terrain.obstacle import Obstacle
import functions as fun
import collision as col
from graph_alg import costFunc as cf

class Forwarder(Machine):
	"""
	Forwarder class. Designed to travel around in a predefined roadnet and pic up trees.
	Thus G.roadNet has to be initialized. It should be a networkx graph where the degree of
	each node >=2. The graph should also have the origin stored with keyword 'origin'.
	So far only supports a single origin/"dump tree spot"
	
	*Specifications are taken from the komatsu 840.4 machine. It was chosen since it is an allround forwarder that
	should giva a correct representation of a standard machine.
	*The capacity is modeled as a mass restriction, not volume.
	*This should be refined later to include both mass and volume.
	"""
	def __init__(self, name, sim, G):
		Machine.__init__(self, name, sim, G=G, driver=None, mass=10000)
		if not G.roadNet: raise Exception('Forwarder needs a road net to work on')
		self.roadNet=G.roadNet #networkX graph class
		self.treeMoni=Monitor(name="Trees in cargo")
		self.totalTreeMoni=Monitor(name="Trees picked up")
		self.distTot=0 #faster than to inspect monitor every time.
		self.distMoni=Monitor(name="distance traveled")
		self.origin=G.roadNet.graph['origin'] #observe, this is and has to be a tuple.
		self.times={'pickUpTrees':8, 'dumpTrees': 14}
		self.velocities= {'machine':1,'reverse': 0.5, 'crane':1} #m/s not taken from reality
		self.setPos(self.origin)
		self.nextPos=None #only none if at origin
		self.pickup_road=None
		self.trees=0
		self.treeWeight=0
		self.treeVolume=0		
		self.capacity=12000 #kg.. don't care about volume now..
		self.grappleCapacity=0.35 #radius of cylinder that can be gripped
		self.craneMax=7.8
		self.reset()
	def run(self):
		"""PEM"""
		#first, naive algorithm
		while True:
			best=self.find_next_road() #position is at origin at this point. 'best' is an nx edge instance.
			if not best:
				self.sim.stopSimulation()
				break
			nList=self.getRoute(best) #get there from origin, from a correct direction
			while len(nList)>0:
				pos=nList.pop()
				yield hold, self, self.setPos(pos)
			#so, we are now in position at the beginning of the road.
			self.pickup_road=best #we will harvest from this road during this loop
			road=self.pickup_road[2]['r']
			wayHome=None #will later be a list of pos. This initialization enables pickups on way home.
			while len(road.harvestedTrees)>0:
				t=self.getClosestTree()
				if not t: break #no tree was found
				pos=col.closestLinePoint(t.pos, np.array(road.endPoints)) #road position of closest tree.
				yield (hold, self, self.setPos(pos)) #move to point closest to t.
				cmd=self.pickup() #pickup all trees at current position
				for c in cmd:
					if c != False: yield c
				brokeOut=False
				if False in cmd: break #full load.
				elif len(road.harvestedTrees)==0: #some room left, and road is out of trees..
					if wayHome is None: wayHome=self.getRoute(self.origin)#get home again.
					while len(road.harvestedTrees)==0:
						yield hold, self, self.setPos(wayHome.pop()) #move. We shall now be at beginning of next road
						if len(wayHome)==0:
							brokeOut=True #enables "double break"
							break
						self.nextPos=wayHome[-1] #other end of road
						self.pickup_road=(tuple(self.pos),tuple(self.nextPos),self.roadNet.get_edge_data(tuple(self.pos), tuple(self.nextPos))) #and the winner of ugly coding is... 
						road=self.pickup_road[2]['r']					
				if brokeOut: break
					
			#go back to origin, full cargo.
			if wayHome is None: wayHome=self.getRoute(self.origin)#get home again.
			while len(wayHome)>0:
				yield hold, self, self.setPos(wayHome.pop())
			self.reset()
	def getClosestTree(self):
		"""
		returns the closest tree in road
		"""
		closest=None
		dist=1e10
		for t in self.pickup_road[2]['r'].harvestedTrees:
			d=fun.getDistance(t.pos, self.pos)
			if d<dist:
				closest=t
				dist=d
		return closest
	def reached_limit(self,t=None):
		if not t: weight=self.treeWeight
		else: weight=t.weight+self.treeWeight
		if weight>self.capacity: return True
		#elif self.treeVolume+t.volume>self.capacity: return True
		return False
	def pickup(self, treeList=None):
		"""
		picups the trees at the current location.
		Adds to cargo sums of trees and handles all connected to this
		returns False if cargo is full.
		if treeList is given it consists of the trees in range at current pos
		"""
		cmnd=[]
		trees=[]
		if treeList: roadTrees=treeList #faster
		else: roadTrees=self.pickup_road[2]['r'].harvestedTrees
		for t in roadTrees:
			d=fun.getDistance(self.pos, t.pos)
			if d<self.craneMax: #reachable
				t.tmpDist=d
				trees.append(t)
		trees=sorted(trees, key=lambda t: -t.tmpDist) #sort, yayy
		tmpList=[]
		told=None
		breakOut=False
		while True: #until we break out
			try:
				t=trees.pop()
			except IndexError: #empty list
				breakOut=True #but not yet.. at end of list..
				told=None 
			if len(tmpList)==0 or (told and fun.getDistance(t.pos,told.pos)<0.1): #we can grip them together!
				tmpList.append(t)
				told=t
				continue #since told!=None this is valid
			#pickup single tree, or cumulative tree bunch
			if len(tmpList)>0: #pickup the accumulated trees
				for tTmp in tmpList:
					if self.reached_limit(tTmp):
						cmnd.append(False)
						print "cargo full!"
						break
					else:
						self.removeTree(tTmp) #also saves statistics. Saves memory
						tmpList.remove(tTmp) #local variable
				cmnd.append((hold, self, self.times['pickUpTrees']))
			if breakOut: break
			tmpList=[t] #new list.
			told=t
		return cmnd
	def find_next_road(self):
		"""
		locates the next road segment with trees to pickup, and saves it in self.pickup_road. If no road was found, pickup_road == None
		The algorithm will increase in complexity, starting with a simple one.
		The road furthest away, with trees left to pickup, is currently returned
		"""
		best=None
		for road in self.roadNet.edges_iter(data=True):
			r=road[2]['r']
			if len(r.harvestedTrees)>0:
				d=fun.getDistance(r.pos,self.origin)
				r.dist_to_origin=d
				if not best or d>best[2]['r'].dist_to_origin:
					best=road
		return best
	def getRoute(self,road):
		"""
		get the routes to origin or to road. If to road, the node furthest away is returned and
		nextPos is set to the node closest to the origin.
		returns a list of nodes that should be visited.
		Does not really follow the turn restrictions, needs improvements.
		"""
		if not road: raise Exception('road does not have a value')
		ls=[]
		shortest_path=nx.dijkstra_path
		if road==self.origin: #give path to origin
			if fun.ListTupleEquals(self.pos, self.origin):
				return []
			elif not self.nextPos: raise Exception('if not located in origin, nextPos must be given')
			if fun.ListTupleEquals(self.pos, self.nextPos):
				ls.append(self.nextPos) #also works if we are on a graph node
			frm=self.nextPos
			if fun.ListTupleEquals(frm, self.origin):
				if not frm in ls: ls.append(frm)
				ls.reverse()
				return ls
			#remove last visited road.
			eTmp=[self.lastRoadPos, frm]
			eTmp.append(self.roadNet.get_edge_data(eTmp[0], eTmp[1]))
			self.roadNet.remove_edge(eTmp[0], eTmp[1])
			ls+=shortest_path(self.roadNet,frm, self.origin) #dijksta shortest path
			self.roadNet.add_edges_from([tuple(eTmp)])
			self.nextPos=self.origin 
		else: #on our way out in the "road-jungle"
			if tuple(self.pos) in self.roadNet:
				frm=self.pos
			else:
				print self.pos, self.origin, self.pos in self.roadNet
				if not self.nextPos: raise Exception('if not located in origin, nextPos must be given', self.pos)
				ls.append(self.nextPos)
				frm=self.nextPos
			#remove road segment from graph, then add it again.
			road=list(road)
			road[2]=self.roadNet.get_edge_data(road[0], road[1])
			self.roadNet.remove_edge(road[0], road[1])
			if frm==road[0]:
				first=[]
			else:
				first=shortest_path(self.roadNet, frm, road[0])
			if frm==road[1]:
				second=[]
			else:
				second=shortest_path(self.roadNet, frm, road[1])
			self.roadNet.add_edges_from([tuple(road)])
			if cf.sumWeights(self.roadNet,first)<cf.sumWeights(self.roadNet,second):
				shortest=first
				no2=second
			else:
				shortest=second
				no2=first
			ls.extend(no2)
			if len(shortest)==0: #we are on the other side of road already.
				self.nextPos=self.pos
			else:
				self.nextPos=shortest[-1] #should be road[0] or road[1]
			self.lastRoadPos=no2[-1] #as above. Used for directions.
		ls.reverse() #to get a stack from it. nicer to just pop()
		return ls
	def removeTree(self, t):
		"""
		This method is called when the tree is picked up. We save data about it all in this method,
		and from this point on the tree wont occupy memory.
		Updates all statistics connected to the tree.
		"""
		self.trees+=1
		self.treeWeight+=t.weight
		self.treeVolume+=t.vol
		self.pickup_road[2]['r'].harvestedTrees.remove(t)
		self.G.terrain.removeObstacle(t)
		self.sim.trees+=1
		self.treeMoni.observe(self.trees, self.sim.now())
		self.totalTreeMoni.observe(self.sim.trees, self.sim.now())
	def reset(self):
		"""
		Resets all parameters, e.g. number of trees in cargo.
		"""
		self.trees=0
		self.treeWeight=0
		self.treeVolume=0
		self.treeMoni.observe(self.trees, self.sim.now())
		self.logsPos=[]
	def setPos(self,pos):
		"""
		Changes position for machine, and records monitors associated with this. Overrides superclass method
		"""
		dnorm=fun.getDistance(pos, self.pos)
		if self.sim.vis and dnorm >0.00001:
			d=np.array(np.array(pos)-np.array(self.pos))
			th=acos(np.dot(d,np.array([1,0])/dnorm))
			if d[1]<0:
				self.direction=2*pi-th
			else:
				self.direction=th
		if self.moveEvent: self.moveEvent.signal() #tell the world that a movement is about to occur.
		traveltime=self.velocities['machine']*dnorm
		self.pos=pos
		self.distTot+=dnorm
		self.distMoni.observe(self.distTot, self.sim.now())
		return traveltime
	def getNodes(self, pos=None):
		#just return a rectangle
		if not pos: pos=self.pos
		w=3.0
		l=5.0
		dx=w*0.5
		dy=l*0.5
		return [(pos[0]-dx, pos[1]-dy), (pos[0]+dx,pos[1]-dy), (pos[0]+dx, pos[1]+dy), (pos[0]-dx, pos[1]+dy)]
	def draw(self, ax):
		"""
		draws forwarder. This method overrides the Machine draw-method and is much more refined.
		Only used for debugging, so speed is not of interest.
		"""
		#determine direction. Nice to not have to save this during the simulation
		direction=self.direction
		cart=fun.getCartesian
		w1=1.5 #first width
		w2=1.1*w1 #second width, in front of cabin
		w3=1.5*w1 #third width, cabin
		l=9.5
		lref=4.170 #cabin part
		l0=lref/15. #part between cabin and crane
		l1=lref/8. #red part between cabin and crane
		l2=0.6*lref/2. #cabin
		l3=0.4*lref/2.
		l4=lref-l1-l2-l3-l0 #front
		redcolor='#CD0000'
		#wheels.
		w=0.293
		xwheels=abs(-w3/2.-w/1.5)
		r=0.6
		pL=[[xwheels, l0+l1+l2+l3]]
		pL.append([-xwheels, pL[0][1]])
		pL.append([xwheels, l0+l1+l1*0.3])
		pL.append([-xwheels, pL[2][1]])
		y1=pL[2][1]-4.970#4.970 is the distance between wheelpairs
		y2=pL[0][1]-4.970
		pL.append([xwheels, y1]) 
		pL.append([-xwheels, y1])
		pL.append([xwheels, y2])
		pL.append([-xwheels, y2])		
		for wPos in pL:
			dx=w/2
			dy=r
			cL=[]
			origin=cart(wPos, origin=list(self.pos), direction=direction, fromLocalCart=True)
			for c in [(dx,dy), (-dx,dy), (-dx,-dy), (dx,-dy)]:
				cL.append(cart(list(c), origin=origin, direction=direction, fromLocalCart=True))
			ax.add_patch(mpl.patches.Polygon(np.array(cL), closed=True, facecolor='k'))
		#ground structure
		left=[]
		right=[]
		d=0.1
		alpha=100
		wback=w1*0.75
		wax=r*0.5
		y=np.array([-(l-lref), y1-wax,y1-wax, y1+wax,y1+wax, y2-wax, y2-wax, y2+wax,y2+wax, 0, l0,l0+l1+l2,l0+l1+l2+l3,l0+l1+l2+l3+l4+d])
		x=np.array([wback,wback, xwheels*2,xwheels*2, wback, wback, xwheels*2, xwheels*2, wback, wback,w3,w3,w2,w1])*0.5+d
		posList=[]
		for i in range(len(x)):
			posList.append([x[i],y[i]])
		for pos in posList:
			right.append(cart(list(pos),origin=list(self.pos), direction=direction, fromLocalCart=True))
			left.append(cart([-pos[0],pos[1]],origin=list(self.pos), direction=direction, fromLocalCart=True))
		left.reverse() #right first, then left
		c=right+left
		ax.add_patch(mpl.patches.Polygon(np.array(c), closed=True, facecolor='k', alpha=alpha))
		#plot the front part.
		left=[]
		right=[]
		y=l0+np.array([0,l1,l1+l2, l1+l2+l3,l1+l2+l3+l4])
		x=np.array([w3,w3,w3,w2,w1])*0.5
		posList=[]
		for i in range(len(x)):
			posList.append([x[i],y[i]])
		for pos in posList:
			right.append(cart(list(pos),origin=list(self.pos), direction=direction, fromLocalCart=True))
			left.append(cart([-pos[0],pos[1]],origin=list(self.pos), direction=direction, fromLocalCart=True))
		left.reverse() #right first, then left
		c=right+left
		ax.add_patch(mpl.patches.Polygon(np.array(c), closed=True, facecolor=redcolor))
		#plot the cabin. (just a little black strip). Begin by defining cabin corners (using symmetry)
		left=[]
		right=[]
		d=0.4
		y=np.array([l0+l1+d,l0+l1+l2, l0+l1+l2+l3-d])
		x=np.array([w3,w3,w2])*0.5-d
		posList=[]
		for i in range(len(x)):
			posList.append([x[i],y[i]])
		for pos in posList:
			right.append(cart(list(pos),origin=list(self.pos), direction=direction, fromLocalCart=True))
			left.append(cart([-pos[0],pos[1]],origin=list(self.pos), direction=direction, fromLocalCart=True))
		left.reverse() #right first, then left
		c=right+left
		ax.add_patch(mpl.patches.Polygon(np.array(c), closed=True, facecolor='k')) #black part
		left=[]
		right=[]
		posList=[]
		d=d*1.
		y=np.array([l0+l1+d,l0+l1+l2, l0+l1+l2+l3-d])
		x=np.array([w3,w3,w2])*0.5-d
		for i in range(len(x)):
			posList.append([x[i],y[i]])
		for pos in posList:
			right.append(cart(list(pos),origin=list(self.pos), direction=direction, fromLocalCart=True))
			left.append(cart([-pos[0],pos[1]],origin=list(self.pos), direction=direction, fromLocalCart=True))
		left.reverse() #right first, then left
		c=right+left
		ax.add_patch(mpl.patches.Polygon(np.array(c), closed=True, facecolor=redcolor)) #red part
		
		
		#"poles" uses symmetry.
		xpole=1.2*(xwheels)
		yL=[-0.4*1.520, y2+r+0.2, y1-r-0.2, -(l-lref)]
		rpole=0.15
		for ypole in yL: #plot the poles and a small bar between them.
			#first, plot bar with width 2.2*rpol. shaded as ground struct
			cL=[]
			for c in [(xpole,ypole+rpole*0.75), (-xpole,ypole+rpole*0.75), (-xpole,ypole-rpole*0.75), (xpole,ypole-rpole*0.75)]:
				cL.append(cart(list(c), origin=list(self.pos), direction=direction, fromLocalCart=True))
			ax.add_patch(mpl.patches.Polygon(np.array(cL), closed=True, facecolor='k', alpha=alpha*0.25))
			for sign in [-1,1]: ax.add_patch(Circle(cart([sign*xpole,ypole], origin=self.pos, direction=direction, fromLocalCart=True), radius=rpole, facecolor=redcolor))
		#trees:
		for i in range(self.trees): #place a tree
			r=0.2
			color='#5C3317'
			try:
				xmid,l=self.logsPos[i]
				#these are saved, to not get a "scramble" for every frame in a video. We could save this every time we chop a tree, but that would take power from the simulation. 
			except IndexError: #new tree
				l=(abs(yL[-1])-abs(yL[0]))+random.uniform(0,0.5)
				xmid=random.uniform(-xpole+rpole+r, xpole-r-rpole)
				self.logsPos.append((xmid,l))
			cL=[]
			for c in np.array([0,yL[0]])-np.array([(xmid+r,0), (xmid-r,0), (xmid-r,l),(xmid+r,l)]):
				cL.append(cart(list(c), origin=self.pos, direction=direction, fromLocalCart=True))
			ax.add_patch(mpl.patches.Polygon(np.array(cL), closed=True, facecolor=color))
