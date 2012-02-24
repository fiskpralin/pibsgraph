#!/usr/bin/env python
import networkx as nx
import numpy as np
from math import *
import random
import sys
import costFunc as cf
if __name__=='__main__':
	import os, sys #insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
from functions import *
import collision as col

	
def sqGridGraph(L=24, umin=0, umax=0, xyRatio=1, diagonals=False, angle=None, areaPoly=None):
	"""
	Creates a square grid graph with dx=dy=L. Umin and umax is the boundaries for the edge uniform distribution.
	xyRatio =0 creates a square area. xyRatio>1 creates an "x>y" rectangle.
	angle is how the grid is turned in radians. 0=default
	Strategy for angle: setup the "base line" by turning the coordinate system by angle. Do a while loop where y is incremented and decremented until we are out of borders.
	"""	
	if angle>pi/2.: raise Exception('onley angles <pi/2 are defined for grid')
	if not areaPoly:
		raise Exception('if areaPoly is not given, elements must be given.')
	C=L/2. #preference questions, this does not span entirely all of space but is a good compromise
	dx=L
	dy=L
	cart=getCartesian
	digits=3
	#xl=np.arange(0,Nx*dx, dx, dtype=np.float)
	"""The strategy is to first cover an extra large area, and then take away the nodes that are outside.
	for a square area, the maximum "x-distance" is sqrt(2) times the side. This corresponds to angle pi/4 or
	5pi/4. The strategy is to always use this maximum length and then take away the ones outisde the area."""
	d=1/sqrt(2)+0.001
	if areaPoly:
		xmin,xmax,ymin,ymax=polygonLim(areaPoly)
		print xmin,xmax,ymin,ymax
	else:
		Ny=floor(sqrt(elements/xyRatio))
		Nx=Ny*xyRatio
		xmax=Nx*L+L*d
		ymax=Ny*L+L*d
		xmin=-L*d
		ymin=-L*d
		areaPoly=[(xmin,ymin), (xmax, ymin), (xmax,ymax), (xmin,ymax)] #square

	xl=np.arange(xmin,ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float) #hypothenuse
	yl=np.arange(ymin,ceil(sqrt(xmax**2+ymax**2)), dy, dtype=np.float) #hypothenuse in another way
	if not angle: angle=0
	direction=angle+pi/2.
	G=nx.Graph( L=L, type='sqGridGraph', C=C)
	G.graph['lim']=np.array([xmin-0.5*L,xmax+0.5*L, ymin-0.5*L, ymax+0.5*L])
	el=0
	for xloc in xl:
		for yloc in yl:
			if yloc==0 or angle==0: sl=[1]
			else: sl=[1,-1]
			for sign in sl:
				x, y=tuple(cart((xloc,sign*yloc), origin=(0,0), direction=direction, fromLocalCart=True))
				x,y=round(x,digits), round(y,digits)
				if not inside((x,y),areaPoly): continue
				G.add_node((x, y))
				el+=1
				#neighbor 'backwards' in y-dir
				neig=tuple(cart((xloc,sign*(yloc)-dy), origin=(0,0), direction=direction, fromLocalCart=True))
				neig=round(neig[0],digits), round(neig[1],digits)
				if inside(neig, areaPoly): G.add_edge((x,y), neig, weight= L+random.uniform(umin,umax), visits=0, visited_from_node=[], c=0)
				if diagonals and xloc != 0:
					neig=tuple(cart((xloc-dx,sign*(yloc-dy)), origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					if inside(neig, areaPoly): G.add_edge((x,y), neig, weight=L* sqrt(2)*(1+random.uniform(umin,umax)), visits=0, visited_from_node=[],c=0)
					neig=tuple(cart((xloc+dx,sign*(yloc-dy)), origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					if inside(neig, areaPoly): G.add_edge((x,y), neig, weight=L* sqrt(2)*(1+random.uniform(umin,umax)), visits=0, visited_from_node=[],c=0)
				if True or xloc != 0:
					neig=tuple(cart((xloc-dx,sign*yloc), origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					if inside(neig, areaPoly): G.add_edge((x,y),neig, weight=L+random.uniform(umin,umax), visits=0, visited_from_node=[],c=0)
	rem=True
	while rem:
		rem=False
		for n in G.nodes(): #pretty ugly, but a must..
			if G.degree(n)==1:
				rem=True
				G.remove_node(n)
				break
	A=getArea(G)
	elements=el
	G.graph['elements']=el
	G.graph['A']=A
	G.graph['L']=L
	G.graph['Ainv']=1./G.graph['A']
	G.graph['density']=elements/G.graph['A']
	G.graph['overlap']={} #will later be filled.
	return G

def getArea(R):
	"""returns the area"""
	rA=0
	for e in R.edges():
		rA+=sRCov(e,R)
	print "area:", rA
	return rA

def sRCov(e, R):
	"""
	Computes the coverage (in m2, not percent) of the edge e
	only accurate for 90degree intersections.
	"""
	l=cf.edgeLength(e)
	w=R.graph['L']
	rA=l*w
	a=w**2*0.5 #overlap area for 90 degree edges.
	for neigh in e[0], e[1]:
		d=R.degree(neigh)
		if d==2: continue #d=2 most of the time, saves time.
		elif d==3:
			rA-=a*0.333333 #compensates for overlaps
		elif d==4:
			#rA-=2*a*0.33333 #same here.
			rA-=2*a*0.25
		elif d>4:
			print "intersection with more than four roads. this is not fully supported."
			rA-=2*a*0.25  #so long
	return rA
def inside(pos,areaPoly):
	"""
	max is a list with two elements. max[0]==xmax, max[1]==ymax. 
	"""
	return col.pointInPolygon(pos,areaPoly)
def triGridGraph(L=24, umin=0, umax=0, xyRatio=1, angle=None, areaPoly=None):
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
	cart=getCartesian
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
	A=getArea(G)
	elements=el
	G.graph['elements']=el
	G.graph['A']=A
	G.graph['L']=L
	G.graph['Ainv']=1./G.graph['A']
	G.graph['density']=elements/G.graph['A']
	G.graph['overlap']={} #will later be filled.
	return G

