#!/usr/bin/env python
import networkx as nx
import numpy as np
from math import *
import random
import sys
from scipy.interpolate import RectBivariateSpline 
from matplotlib.patches import Polygon

import costFunc as cf
import graph_operations as go
import GIS.GIS as GIS
if __name__=='__main__':
	import os, sys #insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import functions as fun
import collision as col
import draw
import costFunc as cf


class ExtendedGraph(nx.Graph):
	"""
	intended as an extension of networkx graph with some features that we want.

	Grids inherit from this class.

	Need to test the coordinates for terrain against something..
	"""
	def __init__(self, origin=None, globalOrigin=None,areaPoly=None):
		if not areaPoly:
			raise Exception('areaPoly must be given.')
		if not origin:
			origin=(0,0)
		if not globalOrigin:
			globalOrigin=(596120, 6727530) #sweref99..located on map.. nice position..
		self.areaPoly=areaPoly
		self.origin=origin
		self.globalOrigin=globalOrigin
		x,y,z=GIS.readTerrain(globalOrigin=globalOrigin , areaPoly=areaPoly)
		#get a list of how x and y varies.. strictly ascending
		xlist=x[:,0]
		ylist=y[0,:]
		self.interpol=RectBivariateSpline(xlist, ylist, z)

		nx.Graph.__init__(self)
		self.t_x=x
		self.t_y=y
		self.t_z=z
		self.graph['origin']=origin
		self.graph['w']=4 #width of road
		self.graph['overlap']={} #will later be filled. A speedup thing
	def getLineElevationCurve(self,p1,p2, points=10):
		"""
		interpolates from the terrain data and returns line coordinates.
		p1-start point
		p2-end point

		output:
		x - array of x-values along line
		y - array of y-values along line
		"""
		x=np.linspace(p1[0], p2[0], points)
		y=np.linspace(p1[1], p2[1], points)
		return x, y, self.interpol.ev(x,y) #evaluate interpolation at above points.
	
	def draw(self, ax=None):
		"""
		does all the plotting. Should be able to determine if we have terrain data etc.
		"""
		ax=GIS.plotBackground(globalOrigin=self.globalOrigin , areaPoly=self.areaPoly, ax=ax)
		ax=GIS.plot2DContour(self.t_x,self.t_y,self.t_z,ax)
		pol=Polygon(self.areaPoly, closed=True, color='none', ec='k',lw=3, ls='solid')
		ax.add_patch(pol)
		draw.draw_custom(G=self, ax=ax, road_color='b')




class SqGridGraph(ExtendedGraph):
	"""
	A square grid graph with dx=dy=L. Umin and umax is the boundaries for the edge uniform distribution.
	xyRatio =0 creates a square area. xyRatio>1 creates an "x>y" rectangle.
	angle is how the grid is turned in radians. 0=default
	Strategy for angle: setup the "base line" by turning the coordinate system by angle. Do a while loop where y is incremented and decremented until we are out of borders.
	"""
	
	def __init__(self,L=24, umin=0, umax=0, xyRatio=1,origin=None, globalOrigin=None,areaPoly=None, diagonals=False, angle=None):
		
		ExtendedGraph.__init__(self, origin=origin, globalOrigin=globalOrigin,areaPoly=areaPoly)
		C=L/2.
		self.graph['C']=C
		self.graph['L']=L
		self.graph['type']='sqGridGraph'		

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
		xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
	
		xl=np.arange(xmin+C,ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float) #hypothenuse
		yl=np.arange(ymin+C,ceil(sqrt(xmax**2+ymax**2)), dy, dtype=np.float) 

		direction=angle+pi/2.
		self.graph['lim']=np.array([xmin-0.5*L,xmax+0.5*L, ymin-0.5*L, ymax+0.5*L])
		self.graph['areaPoly']=areaPoly
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
					#neighbor 'backwards' in y-dir
					neig=tuple(cart((xloc,sign*(yloc)-dy), origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					if inside(neig, areaPoly): self.add_edge((x,y), neig, weight= L+random.uniform(umin,umax), visits=0, visited_from_node=[], c=0)
					if diagonals and xloc != 0:
						neig=tuple(cart((xloc-dx,sign*(yloc-dy)), origin=(0,0), direction=direction, fromLocalCart=True))
						neig=round(neig[0],digits), round(neig[1],digits)
						if inside(neig, areaPoly): self.add_edge((x,y), neig, weight=L* sqrt(2)*(1+random.uniform(umin,umax)), visits=0, visited_from_node=[],c=0)
						neig=tuple(cart((xloc+dx,sign*(yloc-dy)), origin=(0,0), direction=direction, fromLocalCart=True))
						neig=round(neig[0],digits), round(neig[1],digits)
						if inside(neig, areaPoly): self.add_edge((x,y), neig, weight=L* sqrt(2)*(1+random.uniform(umin,umax)), visits=0, visited_from_node=[],c=0)
					if True or xloc != 0:
						neig=tuple(cart((xloc-dx,sign*yloc), origin=(0,0), direction=direction, fromLocalCart=True))
						neig=round(neig[0],digits), round(neig[1],digits)
						if inside(neig, areaPoly): self.add_edge((x,y),neig, weight=L+random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
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
				d=sqrt((n[0]-self.origin[0])**2+(n[1]-self.origin[1])**2)
				if d<short_dist:
					short_dist=d
					shortest=n
			self.origin=shortest
		self.graph['origin']=self.origin
		self.graph['overlap']={} #will later be filled.
		self.graph['w']=4 #width of road
		A=fun.polygon_area(areaPoly)
		elements=el
		self.graph['elements']=el
		self.graph['A']=A
		self.graph['L']=L
		self.graph['Ainv']=1./self.graph['A']
		self.graph['density']=elements/self.graph['A']
def inside(pos,areaPoly):
	"""
	max is a list with two elements. max[0]==xmax, max[1]==ymax. 
	"""
	return col.pointInPolygon(pos,areaPoly)
def triGridGraph(L=24, umin=0, umax=0, xyRatio=1, origin=None,angle=None, areaPoly=None):
	#triGridGraph(elements,L=1, umin=0, umax=0):
	"""
	* Creates a triangular uniform grid.
	"""
	#C=L*0.5
	if not areaPoly: raise Exception('areapoly has to be given to create grid.')
	digits=3
	C=L/2. #preference questions, this does not span entirely all of space but is a good compromise
	dx=L
	dy=L*round(sqrt(3)/2., digits)
	cart=fun.getCartesian
	#xl=np.arange(0,Nx*dx, dx, dtype=np.float)
	"""The strategy is to first cover an extra large area, and then take away the nodes that are outside.
	for a square area, the maximum "x-distance" is sqrt(2) times the side. This corresponds to angle pi/4 or
	5pi/4. The strategy is to always use this maximum length and then take away the ones outisde the area.
	This is an easy but inefficient algorithm, there is certainly room for speed up if required.
	But when coding this, most of the time is spent somewhere else so it doesn't matter really to optimize this part.
	"""
	xmin,xmax,ymin,ymax=polygonLim(areaPoly)
	x1=np.arange(xmin,ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float)
	x2=np.arange(xmin-dx/2., ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float)
	yl=np.arange(ymin,ceil(sqrt(xmax**2+ymax**2)), dy, dtype=np.float) #hypothenuse in another way
	if not angle: angle=0
	G=nx.Graph( L=L, type='sqGridGraph', C=C)
	G.graph['lim']=np.array([xmin-0.5*L,xmax+0.5*L, ymin-0.5*L, ymax+0.5*L])
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
				G.add_node((x, y))
				el+=1
				if y != 0:
					if index != 0:
						neig=tuple(cart([xloc-dx/2., round(yloc-dy,digits)], origin=(0,0), direction=direction, fromLocalCart=True))
						neig=round(neig[0],digits), round(neig[1],digits)
						if inside(neig, areaPoly): G.add_edge((x,y), neig, weight=L+ random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
					neig=tuple(cart([xloc+dx/2., yloc-dy,digits], origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					if inside(neig, areaPoly): G.add_edge((x,y), neig, weight=L+ random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
				if index != 0:
					G.add_edge(tuple([x,round(y,digits)]),tuple([x-dx, round(y,digits)]), weight=L+random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
	rem=True
	while rem:
		rem=False
		for n in G.nodes(): #pretty ugly, but a must..
			if G.degree(n)==1:
				rem=True
				G.remove_node(n)
				break
	G.graph['overlap']={} #will later be filled.
	G.graph['w']=4
	A=polygon_area(areaPoly)
	elements=el
	G.graph['elements']=el
	G.graph['A']=A
	G.graph['L']=L
	G.graph['Ainv']=1./G.graph['A']
	G.graph['density']=elements/G.graph['A']
	G.graph['areaPoly']=areaPoly
	return G

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
	#now, what's the angle?
	print longest
	return fun.angleToXAxis(longest)
