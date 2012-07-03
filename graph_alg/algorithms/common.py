if __name__=='__main__':
	cmd_folder = os.path.split(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0])[0]
	import os, sys,shutil
	print cmd_folder
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import networkx as nx
from math import *
import matplotlib.pyplot as plt

from graph_alg.grid import *
import graph_alg.graph_operations as go
from graph_alg.draw import *
import graph_alg.costFunctions as cf



def get_shortest_and_second(R,node):
	"""
	returns the shortest and second shortest paths FROM node.

	Does NOT store any data.

	assumes that R has origin info. R is of grid-type

	We have a known bug in the cycle thing. It does not consider all cycles..
	
	NOT tested
	"""
	inf=1e15
	node=node[0] #no data dictionary included
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
	p2=nx.dijkstra_path(R, node, R.origin) #cannot do this single source
	w2=go.sumWeights(R,p2)
	cycle=go.shortestCycle(R,node,cutoff=w2-w1) #shortest cycle not including inf. weight edge
	if cycle:
		altW=w1+go.sumWeights(R,cycle)
		alternative=cycle[:-1]+p1 #-1 to avoid double use of node in path..

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


def update_after_mod(ein,R):
	"""
	updates graph after edge was removed or added. Assumes that routingcost function has stored correct data before.

	Because ein is removed from R, the data dictionary e[2] must be included
	"""
	assert len(ein)==3 #data must be given
	inf=1e15
	e_data=ein[2]
	e=(ein[0], ein[1]) #if data is given, we skip it.

	for nTmp in e_data['visited_from_node']: #each node that visited the removed edge
		P11=nTmp[1]['shortest_path']
		P12=nTmp[1]['second_shortest']
		P21, P22=get_shortest_and_second(R, nTmp)
		"""
		earlier, P21 and P22 were already given. But due to a bug connected to the cycles,
		we have to compute it again here.
		"""
		#P21=nTmp[1]['new_shortest_path']
		#P22=nTmp[1]['new_second_shortest']

		for path, diff in (P11, -1), (P12, -1), (P21, 1), (P22,1):
			#1 means add to visits, -1 means subtract
			for eTmp in R.edges_from_path_gen(path, data=False):
				if eTmp==e or eTmp==(e[1],e[0]): continue #this is the removed edge.
				assert R.has_edge(*eTmp)
				d=R.get_edge_data(*eTmp)
				if diff==-1 and nTmp in d['visited_from_node']: #if not, it has already been removed.
					d['visited_from_node'].remove(nTmp)
				elif not nTmp in d['visited_from_node']:
					d['visited_from_node'].append(nTmp)
		#update shortest path info
		assert P21 != None #should have resulted in an infinite cost... and no removal.
		assert P22 != None #otherwise cost should be inf.
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

def pathsDiff(R,e,storeData=False):
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
		P11,P12=get_shortest_and_second(R, nTmp)
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
		C+=(beta*w21+w22)-(beta*w11+w12)
		if storeData: #store so we don't have to do this all again when/if removing edge
			assert P21!=None and P22!=None
			nTmp[1]['new_shortest_path']=P21
			nTmp[1]['new_second_shortest']=P22
		e[2]['weight']=wstore
	if C>=inf: e[2]['weight']=wstore #we broke out without adding..
	return C

def forcePaths(R, diffmax=0.1):
	"""
	This is a warmup function. It modifies the edges weights with respect to how many visits they have and in that way forces the paths to coincide more.

	Ends with restoring the weight.

	Observe that this modification may cause problems in your algorithm..
	"""
	modified=True
	itmax=10
	i=0
	while modified and i<itmax:
		i+=1
		print "forcePath iteration no:", i
		modified=False
		for e in R.edges(data=True):
			e[2]['weight']=R.edgeWeightCalc(e[0],e[1])*(1-min(diffmax,len(e[2]['visited_from_node'])/len(R)))
		#now update paths etc.
		for node in R.nodes(data=True):
			p1,p2=get_shortest_and_second(R,node)
			if len(p1)==0: #origin
				assert node[0]==R.origin
				continue
			node[1]['second_shortest']=[]
			if p1 != node[1]['shortest_path'] or p2 != node[1]['second_shortest']:
				modified=True #we have a change
				#change stored data
				for p in node[1]['shortest_path'], node[1]['second_shortest']:
					for edge in R.edges_from_path_gen(p):
						d=R.get_edge_data(*edge)
						if node in d['visited_from_node']:
							d['visited_from_node'].remove(node)
				#update new info
				node[1]['shortest_path']=p1
				node[1]['second_shortest']=p2
				for p in p1,p2: #now, update edge info for all visited edges
					for edge in R.edges_from_path_gen(p):
						d=R.get_edge_data(*edge)
						if not node in d['visited_from_node']: d['visited_from_node'].append(node) #in order to later see...
	for e in R.edges(data=True):
		e[2]['weight']=R.edgeWeightCalc(e[0],e[1])
	

