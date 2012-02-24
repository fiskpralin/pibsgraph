#!/usr/bin/env python
import functions as fn
import pylab
import numpy as np
import matplotlib.patches as patches
import collision as col
###################################################
# Road
##################################################
class Road(object):
	"""
	Used to make colission detection with trees for strip roads. Could later be
	extended as a resource, to handle traffic jams with several vehicles.
	Only supports straight road segments right now I think...
	"""
	def __init__(self, pos=None, nodes=None, G=None, direction=None, endPoints=None, width=4, radius=None):
		if not nodes and not endPoints: raise Exception('too little information about road. Endpoints or nodes is needed')
		self.G=G
		self.direction=direction #if none, fixnodes will take care of it
		if not nodes: #construct from endpoints and width
			nodes=self._fixNodes(endPoints, width)
		elif not endPoints: #construct from nodes
			if len(nodes)<3: raise Exception('Road instance must have more than two edges.')
			#if not direction: raise Exception('if only nodes are given for road, direction is needed as well.')
		self.nodes=nodes
		self.endPoints=endPoints
		self.color='g'
		self.isSpherical=False
		if not pos: pos= self.getPos() #pretty time consuming...
		self.pos=pos
		if radius:
			self.radius=radius
		else:
			self.radius=-1
			for n in self.nodes:
				d=fn.getDistance(n,self.pos)
				if d>self.radius:
					self.radius=d
		if endPoints: self.length=fn.getDistance(self.endPoints[0], self.endPoints[1])
		self.harvestedTrees=[]
	def _fixNodes(self, endPoints, w):
		"""get nodes from endpoints. depending on the order in endpoints, the nodes are ordered different, but always correct"""
		#r=np.array(endPoints[0])-np.array(endPoints[1])
		[p2,p1]=endPoints
		#xhat=np.array((1,0)) #norm ==1
		r, th=fn.getCylindrical(p1,origin=p2)
		if not self.direction: self.direction=th
		cart=fn.getCartesian
		dx=w*0.5
		nodes=[cart([dx, 0],origin=p2, direction=th,fromLocalCart=True)]
		nodes.append(cart([-dx, 0],origin=p2, direction=th,fromLocalCart=True))
		nodes.append(cart([-dx, r],origin=p2, direction=th,fromLocalCart=True))
		nodes.append(cart([dx, r],origin=p2, direction=th,fromLocalCart=True))
		return nodes
	def getPos(self):
		#returns middle of road
		if not self.endPoints: #find intersection of nodes..
			r1=np.array([self.nodes[0], self.nodes[2]])
			r2=np.array([self.nodes[1], self.nodes[3]])
			tmp, pos=col.linesIntersect(r1,r2, getPoint=True)
			pos=list(pos)
		else:
			
			p2=np.array(self.endPoints[1])
			d=np.array(self.endPoints[0])-p2
			pos=list(p2+0.5*d)
		return pos
	def getNodes(self,pos=None):
		"""required for colission detection"""
		if not pos: pos=self.pos
		elif pos != self.pos: raise Exception('Road cannot be moved')
		return self.nodes
	def draw(self, ax):
		p=patches.Polygon(np.array(self.nodes), closed=True, facecolor=self.color, alpha=200)
		ax.add_patch(p)

