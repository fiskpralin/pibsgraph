from sim.tools import SimSeries
from sim.tools import SimExtend
from sim.tools import globalVar

import terrain
from terrain.terrain import Terrain
import functions as fn
from road import Road

import numpy as np
import copy
import random
import networkx as nx
import collision as col
import time
#from math import *


class Harvester():
	"""harvests 40% of the trees. Not implemented as a machine now, just some routines."""
	def __init__(self, G=None):
		if not G or not G.terrain: raise Exception('Harvester has no information about the terrain.')
		self.name='harvey'
		self.G=G
		self.reach=12 #12m.. in reality its smaller but harvester moves more into the stand.
		self.roadWidth=4.0
		self.run()
	def run(self):
		tic=time.clock()
		rN=self.G.roadNet
		for e in rN.edges_iter(data=True):
			r=e[2]['r']
			direction=r.direction
			tList=self.G.terrain.GetTrees(r.pos, self.reach+r.radius, onlyNonHarvested=True)
			ray=np.array(r.endPoints)
			for t in tList:
				p1, tval=col.closestLinePoint(t.pos, ray, additionalInfo=True) #tval will be used later,
				if col.collide(t, r):
					self.chop(t, p1, tval, r) #could get into problems if t is on ray... hmm..
				elif t.marked and fn.getDistance(p1, t.pos)<=self.reach: #marked foo thinning
				   	self.chop(t, p1, tval, r)
		print "harvester routine took %.4f seconds"%(time.clock()-tic)
	def chop(self,t,pos,tval,road):
		"""chops the tree and places it at pos with direction ~dir	"""
		self.dumpTree(t,pos,tval,road)
		t.harvested=True
	def dumpTree(self, tree, pos, tval, road):
		"""releases the trees at the current position. pos is in the middle of the road, but dumps at the side."""
		if tree.getNodes():
			raise Exception('Tree has already been choped..')
		dir=road.direction
		cart=fn.getCartesian
		#get the dump-pos:
		L=road.length
		l=tree.h
		w=self.roadWidth
		#so, we want it on the long edge of the road, w/2+l/2 from the extreme
		edge=l*0.5+w*0.5
		a=edge/float(L) #the ratio. if a>0.5 we cannot place trees beautifully..
		tlim=[a, 1-a]
		[pl, pm] =road.endPoints 
		if a<0.5:
			if tval>=tlim[1]: #over the limit, tval= np.dot(n, C-A)/(np.dot(n, B-A)) linear..
				#skip the side for now, just put it at a good position.
				p=cart([-w*0.6, -edge], direction=dir, fromLocalCart=True, origin=pm)
			elif tval<=tlim[0]: #over the limit on the other side of the road
				p=cart([-w*0.6, edge], direction=dir, fromLocalCart=True, origin=pl)
			else:
				p1=np.array(pos)
				tp=np.array(tree.pos)
				d=tp-p1
				p=list(p1+0.6*w*d/fn.getDistance(p1,tp)) #pretty beautiful. This is the dump spot. 0.6 to get beside the road
		else:
			p=cart([-w*0.6, 0], direction=dir, fromLocalCart=True, origin=road.pos)
		tree.isSpherical=False
		tree.color='#5C3317' #brown, same as stumps
		dth=np.pi*0.05
		direct=random.uniform(dir-dth, dir+dth)
		r=tree.radius
		ainv=0.5
		tree.pos=p
		c1=cart([-r, l*ainv], origin=p, direction=direct, fromLocalCart=True)
		c2=cart([-r, 0], origin=p, direction=direct, fromLocalCart=True)
		c3=cart([r, 0], origin=p, direction=direct, fromLocalCart=True)
		c4=cart([r, l*ainv], origin=p, direction=direct, fromLocalCart=True)
		tree.nodes=[c1,c2,c3,c4]
		tree.radius=np.sqrt(r**2+l**2)
		road.harvestedTrees.append(tree)
