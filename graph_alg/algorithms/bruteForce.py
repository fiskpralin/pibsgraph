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
from graph_alg.graph_operations import get_shortest_and_second, sortRemList, remove_edge, forcePaths
from graph_alg.draw import *
from functions import getDistance


"""
This is mostly old code and unnecessary complicated. Instead, look inte simplified_bruteForce.

This code is kept since it has some functionality that was not continued with, such as add functionality.

I don't remember why now, but it does not seem to support triangle grids... 

"""


def bruteForce(R, G=None, ax=None, aCap=0.25, beta=1.5, add=True):
	"""
	works with cycles instead of shortest paths

	VERY computationally complex. The earlier algorithms could handle much bigger systems.
	
	modifies G into R:... this is a strange procedure and should be changed.
	"""
	R.beta=beta	
	if not G: G=copy.deepcopy(R)

	inf = 1e15
	eps=1e-9
	lastAdded=None
	origin=G.origin
	if not origin: raise Exception('need info about origin')
	#first, modify the weight of the edges a couple of times. Warmup
	warmUp(R)
	paths=nx.algorithms.shortest_paths.weighted.single_source_dijkstra(R, origin)
	#now, start for real and save away all kind of info about the paths.
	for node in R.nodes(data=True):
		p1=paths[1][node[0]]
		p1.reverse()
		node[1]['shortest_path']=p1
		if len (p1)<=1:
			node[1]['second_shortest']=p1
		else:
			e=[p1[0], p1[1]] #edge closest to point
			e.append(R.get_edge_data(e[0], e[1]))
			R.remove_edge(e[0], e[1]) #temporary remove to get loop
			p2=nx.dijkstra_path(R,origin, node[0])
			node[1]['second_shortest']=p2
			R.add_edges_from([tuple(e)]) #reset road
			#ax=testRoads(R, p1, p2, ax) #used for debugging
			for path in p1,p2:
				last=None
				for nTmp in path:
					if last:
						d=R.get_edge_data(*(last, nTmp)) #should always exist if designed properly
						d['visits']+=1
						if not node in d['visited_from_node']: d['visited_from_node'].append(node)
					last=nTmp
	remList=[]
	addList=[]
	for eTmp in R.edges(data=False):
		e=copy.copy(eTmp) #copy, so we can remove them and then add them and so on.
		e2=R.get_edge_data(*e) #not a copy, reference to real dict.
		e=list(e)
		e.append(e2)
		e[2]['origin_dist']=distToOrigin(e,R)
		e[2]['c']=routingCost(R, e, storeData=False)
		remList.append(e)
	i=1
	#at this point, the visited thing should be updated
	for edge in R.edges(data=True):
		modifyEdge(edge,R,reset=True) #reset to real weight.
	while len(remList)>0:
		i=i+1
		print "start"
		first=True
		e1=False
		assert len([e for e in R.edges(data=True) if e[2]['weight']<0])==0
		while first or e[0:2]!=e1[0:2]:
			e1=remList[0] #takes the last item in the list.
			e1[2]=R.get_edge_data(e1[0], e1[1]) #edgelist is a copy, this is not.
			c=routingCost(R, e1, storeData=True) #also updates e[2]['c']
			#e1[2]=R.get_edge_data(e1[0], e1[1])
			e1[2]['c']=c #-e1[2]['origin_dist']*0.1 #origin dist just to experiment.
			if c>=inf:
				remList.remove(remList[0]) #cannot be empty at this time
				if len(remList)==0: break
			remList=sorted(remList, key=lambda edge: edge[2]['c'])#+edge[2]['visits'])#-edge[2]['origin_dist']*0.1) #first sort
			e=remList[0] # could be the same..
			if first: first=False
		print "the chosen one:",e[2]['c'], e[0:2]
		print "areacover:", R.areaCover
		print "cost:", cf.totalCost(R)
		e[2]=R.get_edge_data(*e) # Update again, to get "new second shortest"
		if add and random.uniform(0,1)<0.33:
			print "adds"
			added, addList, remList, lastAdded=addListProcedure(addList,remList,R,e[2]['c'],i,lastAdded)
			if added: continue #go up to while again
		if len(remList)==0: break
		#print "will remove:", remList[0][0:2]
		remList.remove(remList[0]) #we know now that no edge is added this "round"
		#print "is it still there?"
		#for r in remList:
		#	print r[0:2]
		#print "removed, next", remList[0][0:2]
		print e[2]['c']
		if e[2]['c']>eps and aCap and R.areaCover-go.singleRoadSegmentCoverage(e, R, remove=True)*G.Ainv<aCap:
			print "tries to exit", e[0:2], "ec:", e[2]['c']
			if add:
				added, addList, remList, lastAdded=addListProcedure(addList,remList,R,e[2]['c'],i,lastAdded)
				if added:
					remList.append(e) #since we just removed it and didn't remove it from graph
					continue
			break #we are finished
		if R.degree(e[0])>2 and R.degree(e[1])>2 and c<inf: #loop condition, at least degree 3
			print "removes edge ",e[0:2]
			remove_edge(e, R) #remove from R.
			addList.append(e) #add to lists for potential adding again.
			e[2]['c']*=-1 #reverse, we now gain c by adding it again.
			assert e[2]['c']<=0
			e[2]['i_added']=i
			#this procedure can most certainly be speeded up, expensive operations.
			update_after_mod(e,R)
	for e in R.edges(data=True):
		modifyEdge(e, R, reset=True)
	R.cost=cf.totalCost(R)
	print "construction finished."
	print "total number of nodes:", len(R.nodes())
	print "road area coverage:", R.areaCover
	print "total area:", G.A
	print "total cost:", R.cost
	return R



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
			cycle=go.shortestCycle(R,nTmp[0],cutoff=10*e[2]['weight']) #the cutoff is taken from nothing. Makes the code slow. Should be modified to similar as graphOperations.pathdiff.. but this code is however not used..
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
			assert c>=-1 #should always be.. right?
			if c<0: c=0 #a little bug..c=-0.1 don't use this code anymore so
			#use this ugly solution...
		except:
			print e[0:2], c
			R.draw()
			plt.show()
			raw_input('df')
			raise Exception('dfsd')
		e[2]=R.get_edge_data(e[0], e[1]) #sumPathsDiff takes away e from R, thus we need to update to have the
	#right references
	return c/len(R.nodes())


def distToOrigin(e,R):
	a=np.array(e[0])
	b=np.array(e[1])
	d=a-b
	middle=b+0.5*d
	o=R.origin
	return sqrt((middle[0]-o[0])**2+(middle[1]-o[1])**2)

def warmUp(R):
	for i in xrange(10):
		paths=nx.algorithms.shortest_paths.weighted.single_source_dijkstra(R, R.origin)
		for node in R.nodes(data=True):
			p1=paths[1][node[0]]
			p1.reverse()
			if len(p1)<=1: continue
			e=[p1[0], p1[1]] #edge closest to point
			e.append(R.get_edge_data(e[0], e[1]))
			R.remove_edge(e[0], e[1]) #temporary remove to get loop
			p2=nx.dijkstra_path(R,R.origin, node[0])
			R.add_edges_from([tuple(e)]) #reset road
			for path in p1,p2:
				last=None
				for nTmp in path:
					if last:
						d=R.get_edge_data(*(last, nTmp)) #should always exist if designed properly
						d['visits']+=1
					last=nTmp
		#update edge weight
		for edge in R.edges(data=True):
			modifyEdge(edge,R)
			edge[2]['visits']=0 #zero up.


def modifyEdge(edge, R, reset=False):
	"""
	used systematically in order to deal with the "taxi-cab-geometry-problem", i.e. there
	are several paths of equal length between two specific points.
	"""
	if reset:
		edge[2]['weight']=R.edgeWeightCalc(edge[0], edge[1])
	else:
		edge[2]['weight']=R.edgeWeightCalc(edge[0], edge[1])*(1-float(edge[2]['visits'])/(4.0*float(R.elements)))
		
def addListProcedure(addList,remList, R, c,i,lastAdded=None):
	"""
	does some stuff connected to addList

	This is the thing that adds complexity. 98% of the execution time is spent here.. insane..
	"""
	#print "goes into loop..."
	#routingcost is really expensive, procedure to minimize number of calls.
	for aTmp in addList:
		cTmp=routingCost(R,aTmp,storeData=False, add=True)
		if abs(cTmp)<c and i-aTmp[2]['i_added']>1000: addList.remove(aTmp) #strange procedure, modify later.
		else: aTmp[2]['c']=cTmp
	if len(addList)!=0:
		addList=sorted(addList, key=lambda edge: edge[2]['c']) #first sort
		if len(addList)>2: assert addList[0][2]['c']<=addList[1][2]['c']
		a=addList[0]
		eps=1e-9 #tolerance
		if a[2]['c']<-eps and abs(a[2]['c'])>c+eps: #add edge again
			if lastAdded and lastAdded==a[0:2]:
				print 'noWay'
				return False, addList, remList, lastAdded
			lastAdded=a[0:2]
			print "adds. e:", a[0:2], " ec:", c, "benefit:", a[2]['c']
			addList.remove(a)
			#for at in addList:
			#	if at[0:2]==[(120.0, 168.0), (120.0, 144.0)]: raise Exception('edge was never removed..')
			a[2]['c']*=-1 #you don't get any benefit from it now.
			assert a[2]['c']>=0 #easy to get it wrong..
			add_edge(a, R)
			remList.append(a)
			#print i, "1R has (312.0, 96.0), (312.0, 120.0):", R.has_edge((312.0, 96.0), (312.0, 120.0))
			return True, addList, remList, lastAdded #next while cycle
	return False, addList, remList, lastAdded



def update_after_mod(e,R):
	"""
	updates graph after edge was removed or added. Assumes that routingcost function has stored correct data before.
	"""
	for nTmp in e[2]['visited_from_node']: #each node that visited the removed edge
		P1=nTmp[1]['second_shortest']+nTmp[1]['shortest_path']
		P2=nTmp[1]['new_second_shortest']+nTmp[1]['new_shortest_path']
		#ax=testRoads(R, nTmp[1]['new_shortest_path'], nTmp[1]['new_second_shortest'], ax) #used for debugging
		for path, diff in (P1, -1), (P2, 1):
			last=path[0]
			for nTmp2 in path[1:]: #loop over the edges in path
				if nTmp2==last: continue #overlap between roads, skip
				try:
					d=R.get_edge_data(*(last, nTmp2))
					d['visits']=d['visits']+diff
					if diff==-1 and nTmp in d['visited_from_node']:
						d['visited_from_node'].remove(nTmp)
					elif not nTmp in d['visited_from_node']:
						d['visited_from_node'].append(nTmp)
				except ValueError:
					print "remove failed", d['visited_from_node']
					print d['visited_from_node'].count(nTmp),nTmp
					raise Exception('sads')
				except:
					if not ((last==e[0] and nTmp2==e[1]) or (last==e[1] and nTmp2==e[0])): #if not the removed edge
						raise Exception('tries to modify edge that does not exist, something is wrong')
				last=nTmp2
		nTmp[1]['shortest_path']=nTmp[1]['new_shortest_path']
		nTmp[1]['second_shortest']=nTmp[1]['new_second_shortest']

def remove_edge(e, R):
	"""removes edge e from R and updates related statistics"""
	R.remove_edge(e[0], e[1])
	update_after_mod(e,R)

def add_edge(e, R):
	"""adds e to R and updates statistics."""
	R.add_edges_from([tuple(e)])
	a=R.get_edge_data(e[0], e[1])
	a['c']=routingCost(R,e,storeData=True)
	#update_after_mod(e,R)
	assert a['c']>=0 #who knows..
	
if __name__ == '__main__':
	vis=True
	L=24
	origin=(0.0,0.0)
	Ls=100.0
	#areaPoly=[(0.0,0.0), (2*Ls, 0.0), (2*Ls,Ls),(0.0,Ls)]
	areaPoly=[(0.0,0.0), (3*Ls, Ls), (1.5*Ls,1.5*Ls),(0.0,Ls)]

	R=SqGridGraph(L=L,origin=origin, angle=None, areaPoly=areaPoly, diagonals=False)
	if vis:
		ax=R.draw()
		plot_coverage(R,ax)
	if vis: plt.show()
