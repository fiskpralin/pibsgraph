if __name__=='__main__':
	import sys, os#insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0])[0]
	print cmd_folder
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import networkx as nx
from math import *
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import time
import random
import copy

from graph_alg.grid import *
import graph_alg.costFunctions as cf
import graph_alg.graph_operations as go
from graph_alg.draw import *
from functions import getDistance


def get_shortest_and_second(R,node):
	"""
	returns the shortest and second shortest paths FROM node.

	Does NOT store any data.

	assumes that R has origin info. R is of grid-type
	
	NOT tested
	"""
	inf=1e15
	node=node[0]
	assert len(node)==2 #otherwise node was not given in the way we wanted
	if node==R.origin: return [], []
	p1=nx.dijkstra_path(R, node, R.origin)
	w1=go.sumWeights(R,p1)
	assert len(p1)!=0 #should not happen since origin is handled
	e=(p1[0],p1[1])
	e_data=R.get_edge_data(*e) #need to store away this dictionary of data
	assert node in e #we always start from node.
	wstore=e_data['weight']
	e_data['weight']=inf #to force a loop..
	cycle=go.shortestCycle(R,node) #shortest cycle not including inf. weight edge
	if cycle:
		altW=w1+go.sumWeights(R,cycle)
		alternative=cycle[:-1]+p1 #-1 to avoid double use of node in path..
	p2=nx.dijkstra_path(R, node, R.origin) #cannot do this single source
	w2=go.sumWeights(R,p2)
	if w2>=inf: #if we are forced to use e in second shortest as well -> no p2 or cycle
		if cycle:
			p2=alternative
		else:
			p2=None #corresponds to infinite
	elif cycle and w2>altW:
		p2=alternative #take the loop instead for second shortest
	e_data['weight']=wstore
	return p1,p2
def sortRemList(R,list):
	"""
	sorts remList.
	"""
	costs={}
	for edge in list:
		e_data=R.get_edge_data(*edge)
		costs[edge]=e_data['c']
	return sorted(list, key=lambda edge: costs[edge]) #first sort

def simplified_bruteForce(R, ax=None, aCap=0.25, beta=1.5, add=True):
	"""
	
	"""
	R.beta=beta	
	inf = 1e15
	eps=1e-9
	lastAdded=None
	origin=R.origin
	for e in R.edges(data=True): assert e[2]['weight']>=0
	if not origin: raise Exception('need info about origin')

	#now, start for real and save away all kind of info about the paths.
	for node in R.nodes(data=True):		
		p1,p2=get_shortest_and_second(R,node)
		node[1]['shortest_path']=p1
		if len(p1)==0: #origin
			assert node[0]==origin
			node[1]['second_shortest']=[]
		else:
			node[1]['second_shortest']=p2
		for p in p1,p2: #now, update edge info for all visited edges
			for edge in R.edges_from_path_gen(p):
				d=R.get_edge_data(*edge)
				d['visited_from_node'].append(node) #in order to later see...

	remList=[]
	for e in R.edges(data=False): #add to remove list and calc. costs
		e_data=R.get_edge_data(*e)
		e_data['c']=cost(R, e, storeData=True)
		remList.append(e)
	remList=sortRemList(R,remList)
	
	while len(remList)>0: #the loop where edges are removed..

		"""
		now, choose the edge with the lowest cost.
		We have a problem since the cost saved is not necessary updated to the correct value.
		Actually, the only way to update it correctly is to scan through all edges for every removal. One could think that it would be possible to only update the ones affected by the removal, i.e. the ones connected by the shortest roads. The problem is that we also need to account for the ones that would take this road if some other arbitrary road was removed. Storing that variable would be possible but very memory inefficient. It could be tried and developed further, but would only be necessary if we add edges because the below alg. works pretty good.
		
		We use the fact that the cost from removing edges can only be bigger, i.e when removing edges it only gets worse.
		
		Thus, if the updated cost is still the smallest in the list, we know that this road is cheapest to remove.
		"""
		print "start"
		while True: #two possibilities to break out below..
			e=remList[0]
			e_data=R.get_edge_data(*e)
			e_data['c']=cost(R,e,storeData=False)
			if len(remList)==1:
				break
			#now, look for the other one..
			e2=remList[1]
			e2_data=R.get_edge_data(*e2)
			if e_data['c']<=e2_data['c']:
				break #e is our candidate, we know it's best
			e2_data['c']=cost(R, e2, storeData=False) #update, can only get worse
			for e in e,e2: #check for infinite costs, remove in that case.
				d=R.get_edge_data(*e) #this c has just been updated.
				if d['c']>=inf:
					remList.remove(e)
			remList=sortRemList(R,remList) #sort it, try again..
		if e_data['c']>=inf: break #if last in remList and infinite cost..
		remList.remove(e)
		e_data['c']=cost(R,e,storeData=True) #in order to store the new path..
		#we are outside... will we go over the areaLimit if we remove e?
		if e_data['c']>eps and R.areaCover-go.singleRoadSegmentCoverage(e, R, remove=True)*R.Ainv<aCap:
			assert abs(R.areaCover-go.roadAreaCoverage(R))<eps #compare internal and actual.
			break #we are finished
		assert R.degree(e[0])>2 and R.degree(e[1])>2 #cost func should have given c=inf.
		print "removes edge ",e, e_data['c']
		remove_edge(e, R) #remove from R.
	print "construction finished."
	print "road area coverage:", R.areaCover
	print "total area:", R.A
	return R

def update_after_mod(ein,R):
	"""
	updates graph after edge was removed or added. Assumes that routingcost function has stored correct data before.

	Because ein is removed from R, the data dictionary e[2] must be included
	"""
	assert len(ein)==3 #data must be given
	print "updates after mod"
	inf=1e15
	e_data=ein[2]
	e=(ein[0], ein[1]) #if data is given, we skip it.

	for nTmp in e_data['visited_from_node']: #each node that visited the removed edge
		P11=nTmp[1]['shortest_path']
		P12=nTmp[1]['second_shortest']
		P21=nTmp[1]['new_shortest_path']
		P22=nTmp[1]['new_second_shortest']

		for path, diff in (P11, -1), (P12, -1), (P21, 1), (P22,1):
			#1 means add to visits, -1 means subtract
			assert path[0]==nTmp[0]
			for eTmp in R.edges_from_path_gen(path, data=False):
				if eTmp==e or eTmp==(e[1],e[0]): continue #this is the removed edge.
				assert R.has_edge(*eTmp)
				d=R.get_edge_data(*eTmp)
				if diff==-1:# and nTmp in d['visited_from_node']:
					d['visited_from_node'].remove(nTmp)
				else: #if not nTmp in d['visited_from_node']:
					d['visited_from_node'].append(nTmp)
		#update shortest path info
		nTmp[1]['shortest_path']=P21
		nTmp[1]['second_shortest']=P22
		nTmp[1]['new_shortest_path']=None #to avoid hard-found bugs..
		nTmp[1]['new_second_shortest']=None
def remove_edge(ein, R):
	"""
	removes edge e from R and updates related statistics
	"""
	if len(ein)==2:
		e=(ein[0],ein[1],R.get_edge_data(*ein))
	else:
		e=ein
	R.remove_edge(e[0], e[1])
	update_after_mod(e,R)
	
def cost(R,ein,storeData=False):
	"""
	Calculates the extra routing cost of removing e
	needs edge data, i.e. e[2] should be available. (use: R.edges(data=True))
	
	This cost thing is based on the assumption that every edge fills the forwarder..
	..not really true. 	
	"""
	e_data=R.get_edge_data(ein[0], ein[1])
	e=(ein[0],ein[1],e_data) #subfunctions assume this behaviour. Now we have the right data
	assert R.has_edge(e[0],e[1])
	c=pathsDiff(R,e,storeData)
	assert c>=0 #should always be..
	return c

def pathsDiff(R,e,storeData=False, add=False):
	"""
	culculates the difference in the sum of the paths from removing/adding edge e.
	May store the new paths as well if storeData==True.

	This function is a freaking mess.. clean up..
	"""
	assert len(e)>=3
	beta=R.beta
	w=R.roadWidth #width of roads
	C=0 #cost
	origin=R.origin
	inf=1e15
	eps=1e-8
	etpl=tuple(e)

	for nTmp in e[2]['visited_from_node']: 
		if nTmp[0]==R.origin: continue
		P11=nTmp[1]['shortest_path']
		P12=nTmp[1]['second_shortest']

		#print "remove"
		#p1,p2=get_shortest_and_second(R, nTmp)
		#assert p1==P11 and p2==P12
		
		if P12 == None or P11==None: #road is forced over e.. cannot be removed..
			C=inf
			break
		w11=go.sumWeights(R,P11)
		w12=go.sumWeights(R,P12)
		assert w12>=w11
		wstore=e[2]['weight']
		e[2]['weight']=inf #force other ways..

		P21,P22=get_shortest_and_second(R, nTmp)
		if P21==None or P22==None:
			C=inf
			break
		w21=go.sumWeights(R,P21)
		w22=go.sumWeights(R,P22)
		assert w22>=w21 #closest one is the one with load
		assert w22+w21>=w11+w12 #new route should be longer or equal
		
		C+=(w21+beta*w22)-(w11+beta*w12)
		if storeData: #store so we don't have to do this all again when/if removing edge
			nTmp[1]['new_shortest_path']=P21
			nTmp[1]['new_second_shortest']=P22
		e[2]['weight']=wstore
	if C>=inf: e[2]['weight']=wstore #we broke out without adding..
	return C
	
	
if __name__=='__main__':
	#test_shortest_and_second
	areaPoly=[(0,0), (100,0),(100,100), (0,100)]
	R=SqGridGraph(areaPoly=areaPoly)
	plt.ion()
	ax=None
	for node in R.nodes(data=True):
		print node[0]
		p1,p2=get_shortest_and_second(R,node)
		ax=draw_custom(R,ax=ax, cost=True)
		draw_road(p1, ax, 'b')
		draw_road(p2, ax, 'r')
		plt.draw()
		raw_input()
		ax.clear()
	plt.show()

