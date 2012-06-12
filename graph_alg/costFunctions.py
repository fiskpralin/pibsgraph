#import networkx as nx
from math import *
import matplotlib.pyplot as plt
import numpy as np
from grid import *
import copy
import graph_operations as go
import functions as fun

def sumPathsDiff(R,e,storeData=False, add=False):
	"""
	culculates the difference in the sum of the paths from removing/adding edge e.
	May store the new paths as well if storeData==True.

	This function is a freaking mess.. clean up..
	"""
	#if not storeData: R=copy.deepcopy(R) #why was this here? extremely slow.
	if len(e)<3: raise Exception('sumPathsDiff needs edge data as well. set data=True so e[2] can be reached')
	beta=R.beta
	w=R.roadWidth #width of roads
	C=0 #cost
	origin=R.origin
	inf=1e15
	eps=1e-8
	etpl=tuple(e) 
	if add: #different orders of things, but otherwise the same.
		action1=R.add_edges_from
		action2=R.remove_edges_from
		#we need to reduce the number of nodes checked... if euc. distance
		#to origin is less than half that from e, remove
		dtmp=fun.getDistance(e[0], R.origin)
		lst=[n for n in R.nodes(data=True) if fun.getDistance(n[0], R.origin)>dtmp*0.5]#really slow, but necessary
		#lst=R.nodes(data=True) #old one, replaced to increase speed a bit..
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
		sumP21[n[0]]=go.sumWeights(R, P21) #need to do this before addding/removing edge again
	action2([etpl])
	#print "sumpathdiff, visited from ", len(e[2]['visited_from_node']), " nodes"
	for nTmp in lst:
		if nTmp[0]==R.origin: continue
		#print nTmp[1]
		P11=nTmp[1]['shortest_path']
		P12=nTmp[1]['second_shortest']
		old_s1=go.sumWeights(R,P11)
		#if add and storeData: print "p12:", add, storeData, P12
		old_s2=go.sumWeights(R,P12)
		P21=routeAfter[1][nTmp[0]] #used to be try statement here, be aware of exceptions
		#P21.reverse() #already reversed above now!
		#P21=nx.dijkstra_path(R,nTmp[0], origin) #try single source also..may be faster for a lot of n
		if abs(sumP21[nTmp[0]]- old_s1) > eps: C+=beta*(sumP21[nTmp[0]]-old_s1)
		action1([etpl])
		if C<inf: #road back calculated, now road there.
			eTmp=[P21[0], P21[1]] #minus, since reveresed. Look above
			eTmp.append(R.get_edge_data(eTmp[0], eTmp[1]))
			P21W=go.sumWeights(R,P21) #needs to be calculated before eTmp is removed
			R.remove_edges_from([tuple(eTmp)]) #temporary remove to get loop
			cycle=go.shortestCycle(R,nTmp[0])
			alternative=False
			altW=inf
			W22=None
			if cycle: #if we found one..
				altW=P21W+go.sumWeights(R,cycle)
				tmp=copy.deepcopy(P21)
				tmp.reverse() #want from origin to point
 				alternative=tmp+cycle
			try:
				P22=nx.dijkstra_path(R, origin, nTmp[0]) #cannot do this single source
				if alternative and go.sumWeights(R,P22)>altW:
					P22=alternative
					W22=altW
				else:
					W22=go.sumWeights(R,P22)
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
			#if P22 != old_s2: C+=go.sumWeights(R,P22)-old_s2 #old one. wrong, right?
			if storeData:
				nTmp[1]['new_shortest_path']=P21
				nTmp[1]['new_second_shortest']=P22
		action2([etpl])
	#print "C=", C
	if C>=inf: R.add_edges_from([tuple(e)]) #we broke out without adding again..
	return C


def totalCost(R):
	"""
	prototype, just to get a hint of how good it is..
	"""
	C=0
	for n in R.nodes(data=True):
		C+=R.beta*go.sumWeights(R,n[1]['shortest_path'])
		C+=go.sumWeights(R,n[1]['second_shortest'])
	return C
		
def routingCost(R,e,storeData=False, add=False):
	"""
	Calculates the extra routing cost of removing or adding e.
	Should give a negative value for adding.
	needs edge data, i.e. e[2] should be available. (use: R.edges(data=True))
	"""
	if not add and not R.has_edge(e[0],e[1]):
		print e[0:2]
		raise Exception('e is not in R, if e should be added "add=True" should be set.')
	c=sumPathsDiff(R,e,storeData, add)
	if not add:
		try:
			assert c>=0 #should always be.. right?
		except:
			print e[0:2]
			R.draw()
			plt.show()
			raw_input('df')
			raise Exception('dfsd')
		e[2]=R.get_edge_data(e[0], e[1]) #sumPathsDiff takes away e from R, thus we need to update to have the
	#right references
	return c
