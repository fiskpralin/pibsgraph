#!/usr/bin/env python
import networkx as nx
import numpy as np
from math import *
import random
import sys
from scipy.interpolate import RectBivariateSpline 
from matplotlib.patches import Polygon

import GIS.GIS as GIS
import graph_operations as go
if __name__=='__main__':
	import os, sys #insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import functions as fun
import collision as col
import draw
from weightFunctions import weightFunctionFirst, normalizedPitchDist


class ExtendedGraph(nx.Graph):
	"""
	intended as an extension of networkx graph with some features that we want.

	Grids inherit from this class, with basically just a different constructor.

	Modifies networkX methods for adding and removing edges/nodes so that we can do some more refined statistics for this.

	
	"""
	def __init__(self, origin=None, globalOrigin=None,areaPoly=None, gridtype=None):
		if not areaPoly:
			raise Exception('areaPoly must be given.')
		if not origin:
			origin=(0,0)
		if not globalOrigin:
			globalOrigin=(596120, 6727530) #sweref99..located on map.. nice position.. use as default.
		self.areaPoly=areaPoly
		xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
		side=10
		self.lim=np.array([xmin-0.5*side,xmax+0.5*side, ymin-0.5*side, ymax+0.5*side])
		self.A=fun.polygon_area(areaPoly)
		self.Ainv=1./self.A #used a lot.. faster to just compute this once.
		self.origin=origin
		self.globalOrigin=globalOrigin
		self.type=gridtype
		x,y,z=GIS.readTerrain(globalOrigin=globalOrigin , areaPoly=areaPoly)
		#get a list of how x and y varies.. strictly ascending
		xlist=x[:,0] #just the variations, not the 2D-matrix
		ylist=y[0,:]
		self.interpol=RectBivariateSpline(xlist, ylist, z) #used pretty much everytime we need the height of a specific point. Implemented in fortran and very fast

		nx.Graph.__init__(self)
		self.t_x=x
		self.t_y=y
		self.t_z=z
		self.roadWidth=4 #width of road
		self.overlap={} #will later be filled. A speedup thing
		self.weightFunction=normalizedPitchDist #reference to exter
		self.areaCover=go.roadAreaCoverage(self)
	def getLineElevationCurve(self,p1,p2, points=10):
		"""
		interpolates from the terrain data and returns line coordinates.
		p1-start point
		p2-end point
g
		output:
		x - array of x-values along line
		y - array of y-values along line
		"""
		x=np.linspace(p1[0], p2[0], points)
		y=np.linspace(p1[1], p2[1], points)
		return x, y, self.interpol.ev(x,y) #evaluate interpolation at above points.

	def getPointAltitude(self,p):
		"""
		interpolates from the terrain data and returns point coordinates.

		not tested yet
		"""
		x=np.array(p[0])
		y=np.array(p[1])
		return list(self.interpol.ev(x,y)) #evaluate interpolation at above points.

	def edgeWeightCalc(self,p1,p2):
		"""
		calculates the weight of the edge between p1 and p2.

		So far this is just a simple example, but it should be expanded.

		reason for using self.weightFunction is that several externa functions can be tested..
		"""
		d=fun.getDistance(p1,p2)
		x,y,z=self.getLineElevationCurve(p1,p2, points=max(5, int(d/2))) #every 2 m at least..
		w=self.weightFunction(x,y,z)
		return w
	def remove_node(self,n):
		"""
		like the standard one, but takes away edge first so some data is stored that we need
		"""
		for neigh in self.neighbors(n):
			self.remove_edge(n,neigh)
		super(ExtendedGraph, self).remove_node(n=n)
	def remove_nodes_from(self,nodes):
		"""
		we need to update data, so we only use the above one.
		"""
		for n in nodes:
			self.remove_node(n)
		
	def remove_edge(self, e1, e2):
		"""
		removes edge e from R and updates related statistics
		If more functionality is needed for your road-algorithm, override or write new function
		"""
		super(ExtendedGraph,self).remove_edge(u=e1,v=e2)
		dA=go.singleRoadSegmentCoverage((e1,e2), self, remove=True)
		self.areaCover-=dA*self.Ainv

	def remove_edges_from(self, ebunch):
		"""
		removes edge e from R and updates related statistics
		If more functionality is needed for your road-algorithm, override or write new function
		"""
		for e in ebunch: #one at a time because of overlap-calculations..
			dA=go.singleRoadSegmentCoverage((e[0],e[1]), self, remove=True)
			self.areaCover-=dA*self.Ainv
			super(ExtendedGraph,self).remove_edges_from(ebunch=[e])


	def add_edge(self, e1, e2, attr_dict=None, **kwargs):
		"""
		adds e to edges and updates statistics.
		"""
		e=(e1,e2, kwargs)
		self.add_edges_from([e])

	def add_edges_from(self, ebunch, attr_dict=None, **kwargs):
		"""
		same as above..but for many
		"""
		for e in ebunch: #because of area overlap we need to take one at a time
			super(ExtendedGraph,self).add_edges_from(ebunch=[e], attr_dict=attr_dict, attr=kwargs)
			dA=go.singleRoadSegmentCoverage((e[0],e[1]), self, add=True)
			self.areaCover+=dA*self.Ainv
	def edges_from_path_gen(self, path, data=False):
		"""
		a generator that steps through the edges between the nodes..
		
		e.g.:
		path=[(1,1),(2,2)(3,3)]
		edges=[((1,1),(2,2)), ((2,2),(3,3))]
		
		very handy..
		if data=True, data for each edge is given as well as edge[2].
		Check if the edges exist if data=True

		"""
		if path and len(path) != 0:
			assert len(path) != 1 #one node does not make an edge...
			last=None
			for node in path:
				if last: #not first time..
					if data:
						d=self.get_edge_data(*(last,node))
						yield (last, node, d) #with data. We also know that it exists.
					else:
						yield (last, node) #the edge.. we do not really check if it exists but..
				last=node

			
	def draw(self, ax=None, overlap=False, weight=False, cost=False):
		"""
		does all the plotting. Should be able to determine if we have terrain data etc.
		"""
		ax=GIS.plotBackground(globalOrigin=self.globalOrigin , areaPoly=self.areaPoly, ax=ax)
		ax=GIS.plot2DContour(self.t_x,self.t_y,self.t_z,ax)
		pol=Polygon(self.areaPoly, closed=True, color='none', ec='k',lw=3, ls='solid')
		ax.add_patch(pol)
		draw.draw_custom(G=self, ax=ax, cost=cost,weight=weight, road_color='b', road_width=5)
		if overlap: draw.plot_coverage(self,ax, color='r')
		return ax




class SqGridGraph(ExtendedGraph):
	"""
	A square grid graph with dx=dy=L. Umin and umax is the boundaries for the edge uniform distribution.
	xyRatio =0 creates a square area. xyRatio>1 creates an "x>y" rectangle.
	angle is how the grid is turned in radians. 0=default
	Strategy for angle: setup the "base line" by turning the coordinate system by angle. Do a while loop where y is incremented and decremented until we are out of borders.
	"""
	
	def __init__(self,L=24, umin=0, umax=0, xyRatio=1,origin=None, globalOrigin=None,areaPoly=None, diagonals=False, angle=None):
		
		ExtendedGraph.__init__(self, origin=origin, globalOrigin=globalOrigin,areaPoly=areaPoly, gridtype='sqGridGraph')
		C=L/2.
		self.C=C
		self.L=L

		if angle ==None: #find the longest edge, and use its angle
			print "get the angle"
			angle=getAngleFromLongestEdge(areaPoly, origin)
			print "got the angle.", angle
			while angle>pi/2. or angle<0: #if it is.. most often does not enter loop
				if angle<0:
					angle+=pi/2.
				else:
					angle-=pi/2.
		dx=L
		dy=L
		cart=fun.getCartesian
		digits=3 #we round of to this in order to avoid numerical errors. 1mm is enough :)
		"""The strategy is to first cover an extra large area, and then take away the nodes that are outside.
		for a square area, the maximum "x-distance" is sqrt(2) times the side. This corresponds to angle pi/4 or
		5pi/4. The strategy is to always use this maximum length and then take away the ones outisde the area."""
		d=1/sqrt(2)+0.001
		direction=angle+pi/2.
		xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
		xl=np.arange(xmin+C,ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float) #hypothenuse
		yl=np.arange(ymin+C,ceil(sqrt(xmax**2+ymax**2)), dy, dtype=np.float) 
		el=0
		for xloc in xl:
			for yloc in yl:
				if yloc==0 or angle==0: sl=[1]
				else: sl=[1,-1]
				for sign in sl:
					x, y=tuple(cart((xloc,sign*yloc), origin=(0,0), direction=direction, fromLocalCart=True))
					x,y=round(x,digits), round(y,digits)
					if not inside((x,y),areaPoly): continue
					self.add_node((x, y))
					el+=1
					#neigbor 'backwards' in y-dir
					neig=tuple(cart((xloc,sign*(yloc)-dy), origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					if inside(neig, areaPoly): self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y), neig), visits=0, visited_from_node=[], c=0)
					if diagonals and xloc != 0:
						neig=tuple(cart((xloc-dx,sign*(yloc-dy)), origin=(0,0), direction=direction, fromLocalCart=True))
						neig=round(neig[0],digits), round(neig[1],digits)
						if inside(neig, areaPoly): self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y), neig), visits=0, visited_from_node=[],c=0)
						neig=tuple(cart((xloc+dx,sign*(yloc-dy)), origin=(0,0), direction=direction, fromLocalCart=True))
						neig=round(neig[0],digits), round(neig[1],digits)
						if inside(neig, areaPoly): self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y), neig),visits=0, visited_from_node=[],c=0)
					if True or xloc != 0:
						neig=tuple(cart((xloc-dx,sign*yloc), origin=(0,0), direction=direction, fromLocalCart=True))
						neig=round(neig[0],digits), round(neig[1],digits)
						if inside(neig, areaPoly): self.add_edge((x,y),neig, weight=self.edgeWeightCalc((x,y), neig), visits=0, visited_from_node=[],c=0)
		rem=True
		while rem:
			rem=False
			for n in self.nodes(): #pretty ugly, but a must..
				if self.degree(n)<=1:
					rem=True
					self.remove_node(n)
					break
		if not self.origin in self.nodes():
			shortest=None
			short_dist=1e10
			for n in self.nodes():
				d=fun.getDistance(n, self.origin)
				if d<short_dist:
					short_dist=d
					shortest=n
			self.origin=shortest
		elements=el
		self.elements=el			
		self.density=elements/self.A
def inside(pos,areaPoly):
	"""
	max is a list with two elements. max[0]==xmax, max[1]==ymax. 
	"""
	return col.pointInPolygon(pos,areaPoly)

class TriGridGraph(ExtendedGraph):
	"""
	A triangular grid. Extends from ExtendedGraph
	"""
	def __init__(self,L=24, umin=0, umax=0, xyRatio=1, origin=None, globalOrigin=None,angle=None, areaPoly=None):
		ExtendedGraph.__init__(self, origin=origin, globalOrigin=globalOrigin,areaPoly=areaPoly, gridtype='triGridGraph')
		
		digits=3 #used later..
		C=L/2. #preference questions, this does not span entirely all of space but is a good compromise
		self.L=L
		self.C=C
		dx=L
		dy=L*round(sqrt(3)/2., digits)
		cart=fun.getCartesian
		#xl=np.arange(0,Nx*dx, dx, dtype


		"""The strategy is to first cover an extra large area, and then take away the nodes that are outside.
		for a square area, the maximum "x-distance" is sqrt(2) times the side. This corresponds to angle pi/4 or
		5pi/4. The strategy is to always use this maximum length and then take away the ones outisde the area.
		This is an easy but inefficient algorithm, there is certainly room for speed up if required.
		But when coding this, most of the time is spent somewhere else so it doesn't matter really to optimize this part.
		"""
		xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
		x1=np.arange(xmin,ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float)
		x2=np.arange(xmin-dx/2., ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float)
		yl=np.arange(ymin,ceil(sqrt(xmax**2+ymax**2)), dy, dtype=np.float) #hypothenuse in another way
		if not angle: angle=0
		#G=nx.Graph( L=L, type='sqGridGraph', C=C)
		self.lim=np.array([xmin-0.5*L,xmax+0.5*L, ymin-0.5*L, ymax+0.5*L])
		el=0
		direction=angle+pi/2.

		for yloc in yl:
			if (round(yloc/dy))%2==0:
				xl=x1
				xtype='x1'
			else:
				xl=x2
				xtype='x2'
			for index,xloc in enumerate(xl):
				if yloc==0 or angle==0: sl=[1]
				else: sl=[1,-1]
				for sign in sl:
					x,y=tuple(cart((xloc,sign*yloc), origin=(0,0), direction=direction, fromLocalCart=True))
					x,y=round(x,digits), round(y,digits)
					#x,y is now real coordinates, transformed through angle.
					if not inside((x,y),areaPoly): continue
					self.add_node((x, y))
					el+=1
					if y != 0:
						if index != 0:
							neig=tuple(cart([xloc-dx/2., round(yloc-dy,digits)], origin=(0,0), direction=direction, fromLocalCart=True))
							neig=round(neig[0],digits), round(neig[1],digits)
							if inside(neig, areaPoly): self.add_edge((x,y), neig, weight=L+ random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
							neig=tuple(cart([xloc+dx/2., yloc-dy,digits], origin=(0,0), direction=direction, fromLocalCart=True))
							neig=round(neig[0],digits), round(neig[1],digits)
							if inside(neig, areaPoly): self.add_edge((x,y), neig, weight=L+ random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
					if index != 0:
						self.add_edge(tuple([x,round(y,digits)]),tuple([x-dx, round(y,digits)]), weight=L+random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
		rem=True
		while rem:
			rem=False
			for n in self.nodes(): #pretty ugly, but a must..
				if self.degree(n)==1:
					rem=True
					self.remove_node(n)
					break
		self.overlap={} #will later be filled.
		self.roadWidth=4
		elements=el
		self.elements=el
		self.L=L
		self.density=elements/self.A

def getAngleFromLongestEdge(areaPoly, origin=None):
	"""
	finds the angle of the longest edge in relation to the x-axis.
	if there is a draw, the edge closest to the origin wins.

	This function is only usable for konvex polygons, but works for concave as well.

	the square distance is used since it's faster to compute
	"""
	if not origin: origin=(0,0)
	last=areaPoly[-1]
	longest=None
	dmax=0
	for node in areaPoly:
		d2=fun.getDistanceSq(node, last)
		if d2==dmax: #look for distance to origin
			dtmp=min([fun.getDistanceSq(node, origin), fun.getDistanceSq(last, origin)])
			dtmp2=min([fun.getDistanceSq(node, longest[0]), fun.getDistanceSq(last, longest[1])])
			if dtmp<dtmp2:
				longest = (node,last)
				dmax=d2
		elif d2>dmax:
			longest = (node,last)
			dmax=d2
		last=node
	return fun.angleToXAxis(longest)


