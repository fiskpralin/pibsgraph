import networkx as nx
import costFunctions as cf
import numpy as np
from math import *
"""
A module with a collection of functions connected to road nets and graphs.
"""

def sumWeights(R, P):
	"""
	calculates the sum of the weights in path P.
	"""
	if len(P)<=1: return 0 #should work for this case as well
	C=0
	last=P[0]
	for nTmp2 in P[1:]:
		if nTmp2==last: continue #overlap between roads, skip
		try:
			d=R.get_edge_data(*(last, nTmp2))
			C=C+d['weight']
		except:
			print "start point:", P[0], "endpoint:", P[-1]
			raise Exception("SumWeights: something is wrong. Edge does not seem to exist:",(last, nTmp2))
		last=nTmp2
	return C
def roadAreaCoverage(R):
	"""
	calculate the percentage of the area that is covered by roads.
	"""
	rA=0
	for e in R.edges():
		rA+=singleRoadSegmentCoverage(e,R)
	return rA*R.Ainv

def edgeLength(e):
	"""
	calculates length of edge
	"""
	return sqrt((e[0][0]-e[1][0])**2+(e[0][1]-e[1][1])**2) #pythagora's

def singleRoadSegmentCoverage(e, R, add=False, remove=False):
	"""
	Computes the coverage (in m2, not percent) of the edge e.

	With coverage we mean the area covered by the road, not the area reachable.
	
	only accurate for 90degree intersections.

	add and remove is if the segment is intended to be added or removed. There is a difference regarding how
	the total road area should be modified, related to overlaps.
	"""
	if add or remove:
		modif=1 #count all parts of the road except overlap, that is already fully represented
	else:
		modif=0.5 #count half of the overlap, it is taken into consideration twice.
	l=edgeLength(e)
	A=l*R.roadWidth
	for node, other in [(e[0], e[1]), (e[1], e[0])]:
		for neigh in R.neighbors(node):
			if neigh==other: continue #(node, neigh) is road e, identical road..
			a=overLapA(e, (node, neigh), R)
			A-=a*modif #compensate for overlap.
	return A


def get_angle(e1,e2):
	"""
	returns the angle between roads. angle<=pi, i.e. the smallest one is always chosen.

	Clarification: straight road means angle=pi
	"""
	#first, determine which node that is intersecting
	node=e1[0]
	if node in [e2[0], e2[1]]:
		r1=np.array(e1[1])-np.array(node)
	else:
		node=e1[1] #try the other one...
		if not node in [e2[0], e2[1]]: raise Exception('edges are not connected')
		r1=np.array(e1[0])-np.array(node)
	if node==e2[0]:
		r2=np.array(e2[1])-np.array(node)
	else: #node is e2[1]
		r2=np.array(e2[0])-np.array(node)
	#so, we have two vectors stationed in the origin... the rest is just linear algebra..			
	norm=np.linalg.norm
	d=np.vdot(r1,r2) #this is abs(r1)*abs(r2)*cos(theta)
	arg=d/(norm(r1)*norm(r2))
	if arg>1: #this happens very rarely, some numerical fault.
		arg=1
	elif arg<-1:
		arg=-1
	th=np.arccos(arg)

	th=abs(th)
	if th>pi: th=2*pi-th
	return th

def shortestCycle(R,n):
	"""
	identifies the shortest cycle for undirected graphs
	Returns it.

	Should be able to do this faster without cycle_basis..

	"""
	cycles=[nlist for nlist in nx.algorithms.cycles.cycle_basis(R,n) if n in nlist]
	#identify the shortest of them...
	if len(cycles)==0: return None
	#do some optimization. Weight is not the same for every road BUT it does not differ that much. Saves calls to go.sumWeights
	m=min([len(c) for c in cycles])
	cycles=[c for c in cycles if len(c)<1.5*m] #shortens list.
	weights=[sumWeights(R,P) for P in cycles]
	if min(weights)>=1e15: return None #infinite weight...
	shortest=cycles[weights.index(min(weights))]
	shortest.reverse()
	shortest.append(shortest[0])
	return shortest

def overLapA(e1,e2, R=None):
	"""
	computes the overlapping area of two road segments. Does not handle the dictionary routines at all.

	OBS:
	-this function returns the whole overlap-area. In order to get a correct compensation, half of this area has to be used since it is counted twice.
	-This function gives the overlap, it doesn't account for the fact of "negative overlap", i.e. the area that should be a part of the road but is not counted.
	"""
	assert R != None #must be given
	e1=e1[0:2]
	e2=e2[0:2]
	#first, find out if this calculation has been performed before.
	try:
		overlap=R.overlap
	except: #create dictionary
		R.overlap={} 
		overlap=R.overlap
	#The key structure ((from1, to1), (from2, to2)) gives us four alternatives. Simply try them all.
	#room for optimization...
	keylist=[((e1[0], e1[1]), (e2[0], e2[1])),
			 ((e1[1], e1[0]),(e2[1], e2[0])),
			 ((e2[1], e2[0]),(e1[1], e1[0])),
			 ((e2[0], e2[1]), (e1[0], e1[1]))]
	for key in keylist:
		try:
			a=overlap[key]
			return a
		except KeyError:
			pass #try the next one in keylist
	#we did not find it... calculate instead. Only done once.
	angle=get_angle(e1,e2) 	#get angle between the road segments..
	eps=1e-8
	if angle<eps:
		raise Exception('angle=0 between roads... incident? something must be wrong')
	if abs(angle-pi)<eps: return 0 #straight line-.. no overlaps
	alpha=pi-angle #this is the angle that matters here...
	#the following variables look like "mumbo-jumbo" It's all trigonometry but should be documented somewhere...
	d=0.5*R.roadWidth
	x=2*d*sin(alpha*0.5)
	y=d*cos(alpha*0.5)
	z=0.5*x/tan((pi-alpha)/2)
	a=x*0.5*(y+z)
	overlap[(e1,e2)]=a #save so we don't have to calculate next time
	return a


