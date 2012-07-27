#!/usr/bin/env python
from terrain import *
import functions as fun
import numpy as np
from math import *
import terrain
import copy
import scipy.weave as weave
"""
-Collision detection. Works for 2D objects that inherit from Obstacle, i.e. they have spherical bounding boxes.
-Obstacle class can be find in obstacle.py in terrain folder. 
-If object is not a circle, a method that returns polygon nodes for a given position o.getNodes(pos) is required. Works for 3d if both objects are spheres.
-Rays are assumed to be numpy arrays with a start point and an end point, e.g. np.array([(1,2), (3,4)])
-It would be better with all functions in C, this part of the programs typically take most of the time.

"""
def collide(o1, o2, o1pos=None, o2pos=None):
	"""
	This function is the main function, it categorizes the objects and make the required tests
	"""
	if not o1pos:
		o1pos=o1.pos
	if not o2pos:
		o2pos=o2.pos
	d=fun.getDistance(o1pos,o2pos)
	if d>=o1.radius+o2.radius: #the bounding volumes, spheres/circles in this case
		return False
	if o1.isSpherical and o2.isSpherical:
		return True #collides
	elif o1.isSpherical or o2.isSpherical: 
		#one sphere, one polygon..?
		if o1.isSpherical:
			s=o1
			spos=o1pos
			o=o2
			opos=o2pos
		else:
			s=o2
			spos=o2pos
			o=o1
			opos=o1pos
		nodes=o.getNodes(opos)
		if pointInPolygon(s.pos, nodes): return True
		index=0
		for node in nodes:
			last=nodes[int(index-1)]
			if last != node:
				edge=np.array((last, node))
				if intersectRaySphere(edge, s.radius, spos):
					return True #one of the rays collided.
			index+=1
		return False #all edges tested, does not collide
	else: #two polygons
		#check if o2 is inside o1 and vice versa:
		nodes1=o1.getNodes(o1pos)
		nodes2=o2.getNodes(o2pos)
		if pointInPolygon(nodes2[0], nodes1) or pointInPolygon(nodes1[0], nodes2):
			return True
		index1=0
		for node1 in nodes1: #loop through all the edges in both polygons
			last1=nodes1[int(index1-1)]
			if last1 != node1: 
				ray1=np.array([last1, node1])
				index2=0
				for node2 in nodes2:
					last2=nodes2[int(index2-1)]
					if last2 != node2:
						ray2=np.array([last2,node2])
						if linesIntersect(ray1, ray2): return True
					index2+=1
				index1+=1
		return False #did not collide

def pointInPolygon(p, nodes):
	"""
	test if the point p is inside the polygon defined by nodes. The algorithm used
	is from Ericsson, page 203, and is called "shooting rays"
	"""
	p2=[p[0]+1.0, p[1]+100000.] #we create a point far away in the y-direction. How many times do we cross an edge?
	ray=np.array([p,p2])
	crossings=0
	index=0
	for node in nodes:
		last=nodes[index-1] #this way, the edge between the both extreme points is also considered, nodes[-1] and nodes[0]
		if pointOnLine(ray, node):
			crossings-=1 #will be counted twice later..adjusts
		if node != last: #Ray is not defined for identical points
			edge=np.array((node, last))
			if pointOnLine(edge, p): return True #we are on the border. Have to check this.
			if linesIntersect(edge, ray):
				crossings+=1
		index+=1
	if crossings%2==0: return False #odd no of crossings
	return True

def pointOnLine(ray, p):
	"""
	see Ericsson p.129
	this version uses C-code for speed reasons.
	"""
	A1=float(ray[0][0])
	A2=float(ray[0][1])
	B1=float(ray[1][0])
	B2=float(ray[1][1])
	C1=float(p[0])
	C2=float(p[1])
	code ="""
		double ACAB, ABAB, sqdist,b;
		double AB[2];
		AB[0]=A1-B1;
		AB[1]=A2-B2;
		double AC[2];
		AC[0]=A1-C1;
		AC[1]=A2-C2;
		double BC[2];
		BC[0]=B1-C1;
		BC[1]=B2-C2;
		ACAB=AC[0]*AB[0]+AC[1]*AB[1];
		ABAB=AB[0]*AB[0]+AB[1]*AB[1];
		if (ACAB<=0){
			sqdist=AC[0]*AC[0]+AC[1]*AC[1];
		}
		else if (ACAB>=ABAB){
			sqdist=BC[0]*BC[0]+BC[1]*BC[1];
		}
		else if (ABAB){
			sqdist=AC[0]*AC[0]+AC[1]*AC[1]-ACAB*ACAB/ABAB;
		}
		if (sqdist<0){
			sqdist*=-1;
		}
		if (sqdist<1e-9){
			return_val=1;
		}
		else{
			return_val=0;
		}"""
	a=weave.inline(code, ['A1', 'A2', 'B1', 'B2', 'C1', 'C2'], compiler='gcc')
	if a==0: return False
	else: return True
	
def closestPolygonPoint(pos, nodes):
	"""
	algorithm: go through all the edges, calculate closest

	old note:

	closest point in polygon not defined if point is INSIDE polygon.

	.. I think this is incorrect by inspecting the code. should work inside the polygon as well.

	BUT I have not tested this.. Skip assertions since I will use it for pos inside polygon.

	"""
	d=1e10
	closest=None
	index=0
	for node in nodes:
		last=nodes[int(index-1)]
		edge=np.array((last, node))
		P=closestLinePoint(pos, edge)
		dist=fun.getDistanceSq(pos, P)			
		if dist<d:
			closest=P
			d=dist
		index+=1
	assert d<1e10
	return [closest[0], closest[1]] #not the numpy array, jsut a list

def closestLinePoint(posIn, ray, additionalInfo=False):
	"""
	A startpos, B endpos)
	"""
	a=ray[0]
	b=ray[1]
	pos=np.array(posIn, dtype='float32') #float to avoid integer div. below
	ab=b-a
	t=np.dot(pos-a, ab)/np.dot(ab, ab)
	if t<0:
		if additionalInfo: return a, t
		else: return a
	elif t>1:
		if additionalInfo: return b, t
		else: return b
	else:
		if additionalInfo: return a+t*ab, t
		else: return a+t*ab
		
def linesIntersect(ray1, ray2, getPoint=False):
	"""
	checks if the two lines defined by ray1 and 2 intersect.
	if getPoint=True, the point of intersection for infinite rays is also returned. 
	"""
	A=ray1[0]
	B=ray1[1]
	C=ray2[0]
	D=ray2[1]
	#FIND NORMAL TO CD. 
	n=np.array((-(C[1]-D[1]),C[0]-D[0]), dtype='float32') #one of the two normals
	n=n/sqrt(np.dot(n,n))#normalize
	t=np.dot(n, C-A)/(np.dot(n, B-A))
	if t==np.inf or t==-np.inf:
		p=B-A
		if p[0]==p[1]==0:
			point=A
		else:
			if getPoint: return False, [np.nan, np.nan] #point does not exist, but have to give it
			return False #infinitely far away
	else:
		point=A+t*(B-A)
	if pointOnLine(ray1, point) and pointOnLine(ray2,point):
		if getPoint: return True, point
		return True
	if getPoint: return False, point
	return False
	
def intersectRaySphere(ray, r, pos, additionalInfo=False):
	"""
	check for intersection
	"""
	C=np.array(pos)
	#sphere: (X-C)*(X-C)=r2
	#replace X with R
   	m=ray[0]-C
	d2=ray[1]-ray[0]
	tmax=sqrt(np.dot(d2,d2)) #norm
	d2=d2/tmax
	b=np.dot(m,d2)
	c=np.dot(m,m)-r**2
	d=b**2-c
	if c>0. and b>0.:
		return False #r:s origin outside s and r pointing away from s
	elif d<0:
		return False #ray misses sphere completely
	elif d==0:
		t=-b #one root, tangential hit.
	else:
		t=-b-sqrt(b**2-c) #the smallest root.
		if t<=tmax:
			p1=ray[0]+t*d2
			if t==tmax: #one root
				return True, p1.tolist()
			else:
				t=-b+sqrt(b**2-c)
				if t>tmax:
					return True, p1.tolist()
				p2=ray[0]+t*d2
				return True, p1.tolist(),p2.tolist()
			
	if t>tmax:
		return False #on ray, but ray ended before reaching the sphere
	if additionalInfo:
		p=ray[0]+t*d2
		p.tolist()
		return True, p
	return True #all the test passed. intersects.

def pointInCircle(p, pm, r):
	"""
	pm: middle of circle
	only works for 2D
	"""
	d=fun.getDistance(p,pm)
	if d<=r:
		return True
	else:
		return False
	
def circlesIntersectPoints(P0, P1, r0, r1):
	"""
	taken directyl from stack exchange:

	http://gamedev.stackexchange.com/questions/7172/how-to-find-out-if-two-circles-intersect-each-other
	
	Determines whether two circles collide and, if applicable,
	the points at which their borders intersect.
	Based on an algorithm described by Paul Bourke:
	http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/
	Arguments:
	P0 (complex): the centre point of the first circle
	P1 (complex): the centre point of the second circle
	r0 (numeric): radius of the first circle
	r1 (numeric): radius of the second circle
	Returns:
	False if the circles do not collide
	True if one circle wholly contains another such that the borders
	do not overlap, or overlap exactly (e.g. two identical circles)
	An array of two complex numbers containing the intersection points
	if the circle's borders intersect.
	"""
	if type(P0)==tuple: assert len(P0)==2 #does not support 3d
	assert type(P0)!=complex
	P0=complex(P0[0], P0[1])
	P1=complex(P1[0], P1[1])
	if type(P0) != complex or type(P1) != complex:
		raise TypeError("P0 and P1 must be complex types")
	# d = distance
	d = sqrt((P1.real - P0.real)**2 + (P1.imag - P0.imag)**2)
    # n**2 in Python means "n to the power of 2"
    # note: d = a + b

	if d > (r0 + r1):
		return False
	elif d < abs(r0 - r1):
		return True
	elif d == 0:
		return True
	else:
		a = (r0**2 - r1**2 + d**2) / (2 * d)
		b = d - a
		h = sqrt(r0**2 - a**2)
		P2 = P0 + a * (P1 - P0) / d

		i1x = P2.real + h * (P1.imag - P0.imag) / d
		i1y = P2.imag - h * (P1.real - P0.real) / d
		i2x = P2.real - h * (P1.imag - P0.imag) / d
		i2y = P2.imag + h * (P1.real - P0.real) / d
		
		i1 = complex(i1x, i1y)
		i2 = complex(i2x, i2y)
		return (i1.real, i1.imag), (i2.real, i2.imag)
