#!/usr/bin/env python
from terrain import *
from math import *
import numpy as np
import collision as col
from SimPy.Simulation  import *

def cleanMonitors(sim):
	"""clears some ugly "bumps" of infinitely short timelengths"""
	mon=None
	for mon in sim.allMonitors:
		if len(mon)>2:
			last=mon[0]
			i=1
			for el in mon[1:]:
				if abs(el[0]-last[0])<0.000000001:
					mon.remove(last)
				last=el
	if mon: #throws no error if no monitors have been initialized
		last=[sim.now(),mon[len(mon)-1][1]]
		mon.append(last)
def getDistance(pos1, pos2):
	"""
	returns the distance between two points, has to be cartesian coordinates
	"""
	return sqrt((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)
def getDistanceSq(pos1,pos2):
	"""
	same as above, but squared distance. Saves some time when sqrt is not needed
	"""
	return (pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2
def getCartesian(posCyl,origin=None, direction=None, local=False, fromLocalCart=False):
	"""
	convert from cylindrical to cartesian coordinates
	direction is pretty "unintuitive"... 
	"""
	r=posCyl[0]
	theta=posCyl[1]
	if origin is None:
		#defaultvalues
		x=0
		y=0
	else:
		x=origin[0]
		y=origin[1]
	if direction is None:
		d=pi/2.
	else:
		d=direction
	if fromLocalCart: #pretty ugly.. convert from local cartesian coordinates
		xloc=posCyl[0]
		yloc=posCyl[1]
	else:
		xloc=r*cos(theta) #cartesian coordinates in relation to machine
		yloc=r*sin(theta)
		if local: #used for e.g. collision detection
			return [xloc,yloc]
	cSysDiff=d-pi/2. #the direction of the machine is 90 degrees from x
	#derived from the transition matrix for machine's coordinate system:
	co=cos(cSysDiff)
	si=sin(cSysDiff)
	x=x+co*xloc-si*yloc
	y=y+si*xloc+co*yloc
	return [x,y]
def getCylindrical(pos, origin=None, direction=None):
	"""
	convert from cartesian to cylindrical coordinates
	maybe the geometrical definition of the dot product should be used instead? or numpy internal routines?
	"""
	if origin==None:
		x=0
		y=0
	else:
		x=origin[0]
		y=origin[1]
	if direction is None:
		d=pi/2.
	else:
		d=direction
	r=getDistance(pos, [x,y])
	if r==0: return [0,pi/2.] #angle not defined for origin.
	#translate to just rotational.
	xm=pos[0]-x
	ym=pos[1]-y
	cSysDiff=d-pi/2
	co=cos(cSysDiff)
	si=sin(cSysDiff)
	xloc=co*xm+si*ym
	yloc=-si*xm+co*ym
	#have to split up in cases...
	th=0
	try:
		if xloc>=0:
			th=asin(yloc/r)
		else:
			th=pi-asin(yloc/r)
	except: #abs(argument) was >1, correct it:
		if r>0.00001 and abs(yloc-r)<0.00001: yloc=r #to avoid case when argument for asin is just above zero
		elif r>0.00001 and abs(yloc+r)<0.00001: yloc=-r #same but on the other side
		if xloc>=0:
			th=asin(yloc/r)
		else:
			th=pi-asin(yloc/r)
	return [r,th]
def getAngle(r1, r2):
	"""
	returns the smallest positive radian angle between two rays [p11,p12], [p21,p22]

	not fully tested
	"""
	ray1=np.array(r1)
	ray2=np.array(r2)
	inters,p=col.linesIntersect(ray1,ray2, getPoint=True)
	if inters:
		pts=[r1[0],r1[1], r2[0], r2[1]]
		if not tuple(p) in pts: raise Exception('lines are intersecting and not incident, angle not defined')
	p=np.array(p)
	points=[]
	for ray in ray1,ray2:
		furthestDist=-1
		for point in ray:
			dist=getDistance(p,point)
			if dist>furthestDist:
				furthest=point
				furthestDist=dist
		points.append(point)
	p1=np.array(points[0])-p
	p2=np.array(points[1])-p
	th=acos(np.dot(p1,p2)/(getDistance(p,p1)*getDistance(p,p2)))
	if th>pi:
		if th>2*pi: raise Exception('something is wrong with getAngle')
		th=pi-th
	return th
def getPointBetween(p1,p2):
	"""
	returns the points between points p1 and p2
	"""
	return [(p1[0]+p2[0])*0.5, (p1[1]+p2[1])*0.5]
def angleToXAxis(ray):
	"""
	returns the angle in relation to the xaxis.

	the angle is always positive
	
	"""
	r1,th1=getCylindrical(ray[1], origin=ray[0], direction=pi/2.0)
	r2,th2=getCylindrical(ray[0], origin=ray[1], direction=pi/2.0)
	if th1<0: th1=1e15
	if th2<0: th2=1e15
	assert min(th1, th2)>=0
	return min(th1,th2)

class rect():
	"""
	rectangle
	"""
	def __init__(self, pos, nodes):
		self.pos=pos
		self.radius=0
		for n in nodes:
			d=getDistance(pos,n)
			if d>self.radius:
				self.radius=d
		if self.radius==0: raise Exception('Something is wrong with rectangle nodes')
		self.nodes=nodes
		self.isSpherical=False
	def getNodes(self, pos=None):
		if pos and pos != self.pos: raise Exception('wrong pos for rectangle')
		return self.nodes
class Polygon():
	"""polygon"""
	def __init__(self, pos, nodes):
		self.pos=pos
		self.radius=0
		for n in nodes:
			d=getDistance(pos,n)
			if d>self.radius:
				self.radius=d
		if self.radius==0: raise Exception('Something is wrong with rectangle nodes')
		self.nodes=nodes
		self.isSpherical=False
	def getNodes(self, pos=None):
		return self.nodes
def polygon_area(path):
	"""
	calculates the area of a polygon defined by path.
	"""
	if len(path)<3: raise Exception('polygon_area: polygon must have at least three vertices')
	x=[]
	y=[]
	for n in path:
		x.append(n[0])
		y.append(n[1])
	ind_arr = np.arange(len(x))-1  # for indexing convenience
	s = 0
	for ii in ind_arr:
		s = s + (x[ii]*y[ii+1] - x[ii+1]*y[ii])
	return abs(s)*0.5
def polygonLim(poly):
	"""
	finds the xlim and ylims of polygon p=[(xi,yi), ...]
	returns (xmin, xmax, ymin, ymax)
	"""
	if len(poly)<3: raise Exception('a polygon must have at least three points')
	inf=1e15
	xlim=[inf, -inf]
	ylim=[inf,-inf]
	for p in poly:
		if p[0]<xlim[0]:
			xlim[0]=p[0]
		if p[0]>xlim[1]:
			xlim[1]=p[0]
		if p[1]<ylim[0]:
			ylim[0]=p[1]
		if p[1]>ylim[1]:
			ylim[1]=p[1]
	return tuple(xlim+ylim)
def getPolygonInnerCircle(areaPoly):
	"""
	returns the middle of a polygon, and a radius that is always inside it from this point.
	see reference http://en.wikipedia.org/wiki/Centroid

	Works for all polygons, concave as convex.

	We have to test the radius with every edge all the way, so takes some time.

	NOT carefully tested really..
	"""
	#first, find the point. we have a finite set of points.
	C=np.array([0,0]) #will add stuff to this one..
	for corner in areaPoly: 
		C+=np.array(corner)
	C/=(len(areaPoly))
	C=list(C)

	closest=col.closestPolygonPoint(C,areaPoly)
	return C, getDistance(C,closest)

def insideCircle(pos, c, R):
	"""
	is the points pos inside circle middle c with radius R?
	"""
	d2=getDistanceSq(pos,c)
	if d2<R**2: #could save some calc by using R2, but would give stranger input.
		return True
	else:
		return False

def ListTupleEquals(p1,p2):
	"""
	An ugly and bad way around the fact that we mix tuples and lists, and want to compare their elements.
	An ideal code doesn't use this function
	"""
	for index in xrange(len(p1)):
		try:
			if p1[index]!=p2[index]: return False
		except IndexError: return False #not of equal length
	return True


if __name__ == '__main__':
	#main, test polygon area
	a=polygon_area([(0,0), (2,0), (2,2), (0,2), (0,0)])
	print "expect 4, got %f"%a
	L=sqrt(2)
	a=polygon_area([(0,0), (L,0), (L,L), (0,L)])
	print "expect 2, got %f"%a
	
