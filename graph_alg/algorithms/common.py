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



def get_shortest_and_second(R,node,debug=False):
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
	cycle=go.shortestCycle(R,node, debug) #shortest cycle not including inf. weight edge
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
		"""if __debug__: #if code is not optimized (-o)
			p1,p2=get_shortest_and_second(R, nTmp) #what we save by storing stuff..
			print p1==P21, p2==P22
			if p2 != P22:
				print nTmp
				ax=R.draw(weight=True)
				draw_road(p2, ax, 'r')
				draw_road(P22, ax, 'b')
				plt.show()
			assert p1==P21
			assert p2==P22"""
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
		"""
		used to use already calc. balues..
		P11=nTmp[1]['shortest_path']
		P12=nTmp[1]['second_shortest']
		#remove!!!
		"""
		P11,P12=get_shortest_and_second(R, nTmp)
		"""if t2!=P12:
			print nTmp
			print go.sumWeights(R,t2)
			print go.sumWeights(R,P12)
			print t2
			print P12
			ax=R.draw(weight=True)
			draw_road(t2, ax,'r')
			draw_road(P12, ax,'b')
			plt.show()
		assert t1==P11
		assert t2==P12"""
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
		#print w22, w21, w22+w21
		#print w11, w12, w11+w12
		if w22+w21<w11+w12: #new route should be longer or equal
			print w22+w21, w11+w12
			print e[0:2]
			ax=R.draw()
			P21.reverse()
			draw_road(P21+P22, ax, 'r')
			P11.reverse()
			draw_road(P11+P12, ax, 'b')
			plt.show()
			
		assert w22+w21>=w11+w12 #new route should be longer or equal
		C+=(w21+beta*w22)-(w11+beta*w12)
		if storeData: #store so we don't have to do this all again when/if removing edge
			assert P21!=None and P22!=None
			"""if nTmp[0]==(135.637, 103.395):
				p1,p2=get_shortest_and_second(R,nTmp, debug=True)
				if go.sumWeights(R,p2)>160:
					ax=R.draw(weight=True)
					draw_road(p2, ax, 'r')
					plt.show()
					raise Exception('remove..')"""
			nTmp[1]['new_shortest_path']=P21
			nTmp[1]['new_second_shortest']=P22
		e[2]['weight']=wstore
	if C>=inf: e[2]['weight']=wstore #we broke out without adding..
	return C
