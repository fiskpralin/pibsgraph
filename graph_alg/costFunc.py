#import networkx as nx
from math import *
import matplotlib.pyplot as plt
import numpy as np
from grid import *
import copy
import alg
import graph_operations as go

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
	"""calculate the percentage of the area that is covered by roads."""
	w=R.graph['w']
	rA=0
	a=w*w/2.
	for e in R.edges():
		rA+=singleRoadSegmentCoverage(e,R)
	return rA*R.graph['Ainv']
def edgeLength(e):
	"""calculates length of edge"""
	return sqrt((e[0][0]-e[1][0])**2+(e[0][1]-e[1][1])**2) #pythagora's
def singleRoadSegmentCoverage(e, R, add=False, remove=False):
	"""
	Computes the coverage (in m2, not percent) of the edge e
	only accurate for 90degree intersections.

	add and remove is if the segment is intended to be added or removed. There is a difference regarding how
	the total road area should be modified, related to overlaps.
	"""
	if add or remove:
		modif=1 #count all parts of the road except overlap, that is already fully represented
	else:
		modif=0.5 #count half of the overlap, it is taken into consideration twice.
	l=edgeLength(e)
	A=l*R.graph['w']
	nodes=[e[0],e[1]]
	#new stuff
	for i in [0,1]:
		node=nodes[i]
		if R.degree(node)<=2: continue #we also get a "gap", for d=2 this gap is of equal size as the overlap.
		othernode=nodes[i-1]
		for neigh in R.neighbors(node):
			if neigh==othernode: continue #(node, neigh) is road e, identical road..
			a=go.overLapA(e, (node, neigh), R)
			A-=a*modif #compensate for overlap.
	return A
def roadCost2(R):
	"""calculates the overall transportation cost of a road, normalized with respect to the area."""
	w=R.graph['w'] #width of roads
	C1=0 #cost
	rho=R.graph['density']
	for n in R.nodes(data=True): #may be defined in an other way later.
		P1=n[1]['shortest_path']
		P2=n[1]['second_shortest']
		C1=C1+R.graph['beta']*(sumWeights(R, P1))+sumWeights(R, P2)
	return C1/rho

def roadCost(R):
	"""calculates the overall transportation cost of a road, normalized with respect to the area."""
	w=R.graph['w'] #width of roads
	C1=0 #cost
	C2=0
	rho=R.graph['density']
	Ainv=R.graph['Ainv']
	origin=R.graph['origin']
	inf=1e15
	for n in R.nodes(data=True): #may be defined in an other way later.
		P1=n[1]['shortest_path']
		P2=n[1]['second_shortest']
		C1=C1+R.graph['beta']*(sumWeights(R, P1))+sumWeights(R, P2)
	for e in R.edges():
		d=edgeLength(e)
		C2=C2+R.graph['cr']*d*w/Ainv
	C1=C1*R.graph['cd']/(R.graph['A']**(3/2.)*rho)
	return C1+C2, C1, C2
def shortestCycle(R,n):
	"""
	identifies the shortest cycle for undirected graphs
	Returns it.
	"""
	cycles=[nlist for nlist in nx.algorithms.cycles.cycle_basis(R,n) if n in nlist]
	#identify the shortest of them...
	if len(cycles)==0: return None
	weights=[sumWeights(R,P) for P in cycles]
	shortest=cycles[weights.index(min(weights))]
	shortest.reverse()
	shortest.append(shortest[0])
	#reshuffle, want n in beginning and end
	return shortest

def sumPathsDiff(R,e,storeData=False, add=False):
	"""culculates the difference in the sum of the paths. May store the new paths as well."""
	#if not storeData: R=copy.deepcopy(R) #why was this here? extremely slow.
	if len(e)<3: raise Exception('sumPathsDiff needs edge data as well.')
	beta=R.graph['beta']
	w=R.graph['w'] #width of roads
	C=0 #cost
	origin=R.graph['origin']
	inf=1e15
	eps=1e-8
	etpl=tuple(e) 
	if add: #different orders of things, but otherwise the same.
		action1=R.add_edges_from
		action2=R.remove_edges_from
		lst=R.nodes(data=True)#much slower.. but necessary.
	else:
		action1=R.remove_edges_from
		action2=R.add_edges_from
		lst=e[2]['visited_from_node'] #faster than above.
	action1([etpl])
	if not nx.is_connected(R): #probably remove in this case. Not allowed since graph looses connectivity
		action2([etpl])
		return inf
	routeAfter=nx.algorithms.shortest_paths.weighted.single_source_dijkstra(R, origin)
	sumP21={}
	for n in lst:
		P21=routeAfter[1][n[0]]
		P21.reverse()
		sumP21[n[0]]=sumWeights(R, P21) #need to do this before addding/removing edge again
	action2([etpl])
	#print "sumpathdiff, visited from ", len(e[2]['visited_from_node']), " nodes"
	for nTmp in lst:
		if nTmp[0]==R.graph['origin']: continue
		#print nTmp[1]
		P11=nTmp[1]['shortest_path']
		P12=nTmp[1]['second_shortest']
		old_s1=sumWeights(R,P11)
		#if add and storeData: print "p12:", add, storeData, P12
		old_s2=sumWeights(R,P12)
		P21=routeAfter[1][nTmp[0]] #used to be try statement here, be aware of exceptions
		#P21.reverse() #already reversed above now!
		#P21=nx.dijkstra_path(R,nTmp[0], origin) #try single source also..may be faster for a lot of n
		if abs(sumP21[nTmp[0]]- old_s1) > eps: C+=beta*(sumP21[nTmp[0]]-old_s1)
		action1([etpl])
		if C<inf: #road back calculated, now road there.
			eTmp=[P21[0], P21[1]] #minus, since reveresed. Look above
			eTmp.append(R.get_edge_data(eTmp[0], eTmp[1]))

			P21W=sumWeights(R,P21) #needs to be calculated before eTmp is removed
			R.remove_edges_from([tuple(eTmp)]) #temporary remove to get loop
			cycle=shortestCycle(R,nTmp[0])
			alternative=False
			altW=inf
			W22=None
			if cycle: #if we found one..
				altW=P21W+sumWeights(R,cycle)
				tmp=copy.deepcopy(P21)
				tmp.reverse() #want from origin to point
 				alternative=tmp+cycle
			try:
				P22=nx.dijkstra_path(R, origin, nTmp[0]) #cannot do this single source
				if alternative and sumWeights(R,P22)>altW:
					P22=alternative
					W22=altW
				else:
					W22=sumWeights(R,P22)
			except nx.exception.NetworkXNoPath:
				if alternative:
					P22=alternative
					W22=altW
				else:
					C=inf #this exception is thrown if the node is in a local graph separated from OR
			R.add_edges_from([tuple(eTmp)]) #reset road
			if C>=inf: break #saves some time, but is ugly
			if abs(W22-old_s2)>eps:
				C+=W22-old_s2
			#if P22 != old_s2: C+=sumWeights(R,P22)-old_s2 #old one. wrong, right?
			if storeData:
				nTmp[1]['new_shortest_path']=P21
				nTmp[1]['new_second_shortest']=P22
		action2([etpl])
	#print "C=", C
	if C>=inf: R.add_edges_from([tuple(e)])
	return C
def refinedCost(R,  e, storeData=False):
	"""
	* cost of removing edge e in road net R, does not modify R but saves some shortest
	  path information to save future computations, if storeData is set to True
	* Does not work for directed graphs right now.
	* Only works for "single origin" right now.
	* Needs testing and verification
	"""
	C=sumPathsDiff(R,e,storeData)
	C2=R.graph['cr']*singleRoadSegmentCoverage(e)
	return R.graph['cd']*C/R.graph['density']-C2
def routingCost(R,e,storeData=False, add=False):
	"""
	Calculates the extra routing cost of removing or adding e.
	Should give a negative value for adding.
	"""
	if not add and not R.has_edge(e[0],e[1]):
		print e[0:2]
		raise Exception('e is not in R, if e should be added "add=True" should be set.')
	a=sumPathsDiff(R,e,storeData, add)/R.graph['density']
	if not add: e[2]=R.get_edge_data(e[0],e[1]) #sumPathsDiff takes away e from R, thus we need to update to have the
	#right references
	return a
def cost(G, P1, P2):
	"""
	calculates the cost for replacing P1 with P2 in terrain graph G. Both P1 and P2 are lists of tuples of positions of paths
	OLD
	"""
	costs={}
	for path,ID in [P1,'P1'], [P2,'P2']:
		c=0
		if len(path)>1:
			last=path[0]
			for nTmp in path[1:]:
				if last==nTmp: continue #overlaps between road there and back.
				d=G.get_edge_data(*(last, nTmp))
				c=c+d['weight']
				last=nTmp
		costs[ID]=c
	return costs['P2']-costs['P1']
def roadFuncEval():
	"""
	examines how the road cost function varies with area. It is supposed to be independent of area.
	old stuff, not updated.
	"""
	variable='cr' #options: 'A', 'cr'
	C1=[]
	C2=[]
	Ctot=[]
	A=[]
	crlist=[]
	x=[]
	L=1
	origin=(0,0)
	elements=50 #standard
	cr=50
	it=100
	list=[]
	if variable=='A': #vary elements instead of A
		pv=A
		el=np.linspace(50, 700, it)
		for elements in el:
			list.append((elements, cr))
	elif variable=='cr':
		pv=crlist
		crl= np.logspace(0, 27, it, base=np.e-1) #up to e^17
		for cr in crl:
			list.append((elements, cr))
	else: raise Exception('roadFuncEval: variable %s is not valid'%str(variable))
	for elements, cr  in list:
		G=sqGridGraph(elements, L, umin=0, umax=0.00001)
		G.graph['origin']=origin
		G.graph['beta']=1.25
		G.graph['cd']=1
		G.graph['cr']=cr
		R=copy.deepcopy(G)
		alg.cycleRoad(G,R)
		r=roadCost(R)
		Ctot.append(r[0])
		C1.append(r[1])
		C2.append(r[2])
		A.append(G.graph['A'])
		crlist.append(G.graph['cr'])
		print elements, cr
	fig1=plt.figure()
	ax2=alg.draw_custom(R, edge_visits=True)
	alg.plot_coverage(R,ax2)
	fig=plt.figure()
	ax=fig.add_subplot(111)
	if variable != 'cr': plist=[(C1,'b'),(C2,'c'),(Ctot,'r')]
	else:
		plist= [(C1,'b')]
		pv=np.log(np.array(pv)) #logplot
		variable='log(cr)'
	for C, color in plist:
		ax.plot(pv,C,color)
	ax.set_xlabel(variable)
	ax.set_ylabel('C')
	
