if __name__=='__main__':
	cmd_folder = os.path.split(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0])[0]
	import os, sys,shutil
	print cmd_folder
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import networkx as nx
from math import *
import matplotlib.pyplot as plt
import copy

from graph_alg.grid import *
import graph_alg.graph_operations as go
from graph_alg.graph_operations import get_shortest_and_second, sortRemList, remove_edge, forcePaths
from graph_alg.draw import *
import graph_alg.costFunctions as cf
from graph_alg.costFunctions import cost as cost


def simplified_bruteForce(R, ax=None, aCap=0.20, beta=1.5, warmup=False, anim=False):
	"""
	a simplified algorithm. Still a little bit complex dock, but this is as simple as it gets I think.

	Does not add any edges, only removal. No stochastic elements are involved.

	anim=True creates a movie.
	"""
	R.beta=beta	
	inf = 1e12
	eps=1e-9
	lastAdded=None
	origin=R.origin
	if not origin:
		raise Exception('need info about origin')

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
				if not node in d['visited_from_node']: d['visited_from_node'].append(node) #in order to later see...
	if warmup:
		forcePaths(R)
	remList=[]
	for e in R.edges(data=False): #add to remove list and calc. costs
		e_data=R.get_edge_data(*e)
		e_data['c']=cost(R, e, storeData=True)
		remList.append(e)
	remList=sortRemList(R,remList)

	#all stuff initialized by now.
	while len(remList)>0: #the loop where edges are removed..

		"""
		now, choose the edge with the lowest cost.
		We have a problem since the cost saved is not necessary updated to the correct value.
		Actually, the only way to update it correctly is to scan through all edges for every removal. One could think that it would be possible to only update the ones affected by the removal, i.e. the ones connected by the shortest roads. The problem is that we also need to account for the ones that would take this road if some other arbitrary road was removed. Storing that variable would be possible but very memory inefficient. It could be tried and developed further, but would only be necessary if we add edges because the below alg. works pretty good.
		
		We use the fact that the cost from removing edges can only be bigger, i.e when removing edges it only gets worse.
		
		Thus, if the updated cost is still the smallest in the list, we know that this road is cheapest to remove.
		"""
		if anim: #movietime
			R.movieFlush()
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
		#found our candidate
		if e_data['c']>=inf: break #if last in remList and infinite cost..
		remList.remove(e)
		e_data['c']=cost(R,e,storeData=True) #in order to store the new path..
		assert e_data['c']<inf
		#we are outside... will we go over the areaLimit if we remove e?
		if e_data['c']>eps and R.areaCover-go.singleRoadSegmentCoverage(e, R, remove=True)*R.Ainv<aCap:
			assert abs(R.areaCover-go.roadAreaCoverage(R))<eps #compare internal and actual.
			break #we are finished
		assert R.degree(e[0])>2 and R.degree(e[1])>2 #cost func should have given c=inf.
		print "removes edge ",e, e_data['c']
		remove_edge(e, R) #remove from R.
	if anim:
		R.movieFlush(final=True)
	R.cost=cf.totalCost(R)
	print "construction finished."
	print "road area coverage:", R.areaCover
	print "total area:", R.A
	print "number of nodes", len(R.nodes())
	print "total cost:", R.cost
	return R

