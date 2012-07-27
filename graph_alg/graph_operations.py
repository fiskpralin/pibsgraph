import networkx as nx
import costFunctions as cf
import numpy as np
import copy
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

def shortestCycle(R,source, cutoff=None):
	"""
	Shortest cycle.

	Cutoff is the maximum sum weight of paths we can accept.
	Important to set cutoff as small as possible since it is a very complex algorithm.

	This is a brute force search, takes some time. May be refined.

	When a potential "winner" is found, all the other paths are explored until
	their weight-sum is bigger than the "winner"-one. If a better soulution is
	found in this process, we do the same for that one.

	There's a lot of asserts in here. Remove them if you find the distracting.
	"""
	assert cutoff!=None #otherwise we are screwed..
	assert source in R #hard if n is not in our graph
	if cutoff==0: return None
	paths={0:[source]} # paths in [n1, n2,..] format
	sums={0:0} #sum of weight for path i

	id=0
	current=0 #indicates where in the lists we are
	potential_winner=None 
	pwsum=1e12
	shortest_paths=None #will be calc. and set if needed below.

	while sums[current]<cutoff:
		path=paths[current]
		node=path[-1] #last node in path
		for neigh in R[node]: #neighbors
			if len(path)>1 and neigh == path[-2]:
				continue #we do not allow going back again
			elif len(path)!=1 and neigh in path: #we might have found it.
				if neigh==source: #we are there..
					e_data=R.get_edge_data(node,neigh)
					s=sums[current]+e_data['weight']
					if s>=pwsum: #too bad.. don't fork
						continue
					ptmp=copy.deepcopy(path)
					ptmp.append(neigh)	
				else: #we have performed a loop. Take the shortest path home.
					index=path.index(neigh)
					assert path[-1]==node
					assert index != len(path)-1 #would be node..
					assert index != 0
					if path[index-1] == node:
						continue #we have been down that road, don't try it again
					#simply take the shortest path back.
					if shortest_paths==None: #then calculate it..better to do here
						shortest_paths=nx.algorithms.shortest_paths.weighted.single_source_dijkstra_path(R,source,cutoff=cutoff)
						for i in shortest_paths.keys(): #reverse the paths
							shortest_paths[i].reverse()
					ptmp=shortest_paths[neigh]
					s=sums[current]+sumWeights(R,ptmp)
					if s>=pwsum:
						continue #route is too bad.
					assert ptmp[0]==neigh
					assert path[-1]!=neigh
					assert ptmp[-1]==source
					ptmp=copy.deepcopy(path)+ptmp
					assert ptmp.count(source)==2
					assert ptmp.count(neigh)==2
				potential_winner=ptmp
				pwsum=s
				continue #should not fork this one since we've reached the goal
			#if we have come this far, we should fork, i.e. start a new branch.
			e_data=R.get_edge_data(node, neigh)
			newsum=sums[current]+e_data['weight']
			if newsum>cutoff:
				continue #too much weight
			ptmp=copy.deepcopy(path)
			ptmp.append(neigh)
			id+=1 #this is a unique id 
			sums[id]=newsum
			paths[id]=ptmp
		del paths[current] #we always create new id:s.. so remove
		del sums[current]
		
		if len(paths)==0:
			if potential_winner: return potential_winner
			assert len(sums)==0
			return None

		current = min(sums, key=sums.get) #so, we get the key to the instance with lowest sum for next iteration
		if potential_winner: #we may exit here
			if pwsum<sums[current]:
				return potential_winner #we have checked that no shorter path exists. Bingo.

	return None #we reached the cutoff


def get_shortest_and_second(R,node):
	"""
	returns the shortest and second shortest paths FROM node.

	Does NOT store any data.

	assumes that R has origin info. R is of grid-type

	This function is used heavily. Most of the program time is spent in here.
	"""
	inf=1e12
	node=node[0] #no data dictionary included
	assert len(node)==2 #otherwise node was not given in the way we wanted
	if node==R.origin: return [], []
	p1=nx.dijkstra_path(R, node, R.origin)
	w1=sumWeights(R,p1)
	assert len(p1)!=0 #should not happen since origin is handled
	e=(p1[0],p1[1])
	e_data=R.get_edge_data(*e) #need to store away this dictionary of data
	assert node in e #we always start from node.
	wstore=e_data['weight']
	e_data['weight']=inf #to force a loop..
	p2=nx.dijkstra_path(R, node, R.origin) #cannot do this single source
	w2=sumWeights(R,p2)
	cycle=shortestCycle(R,node,cutoff=w2-w1) #shortest cycle not including inf. weight edge
	if cycle:
		altW=w1+sumWeights(R,cycle)
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
	updates graph after edge was removed or added. Assumes that routingcost function has stored correct data before. I.e. cost has been called with argument storeData=True

	Because ein is removed from R, the data dictionary e[2] must be included
	"""
	if len(ein)<3:
		raise Exception('update_after_mod needs ein[2] data dictionary.' )
	assert len(ein)==3 #data must be given
	assert not ein in R #it must be removed..
	inf=1e12
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
	

def pathsDiff(R,e,storeData=False):
	"""
	culculates the difference in the sum of the paths from removing/adding edge e.
	May store the new paths as well if storeData==True.
	"""
	assert len(e)>=3
	beta=R.beta
	w=R.roadWidth #width of roads
	C=0 #cost
	origin=R.origin
	inf=1e12
	eps=1e-8
	etpl=tuple(e)

	for nTmp in e[2]['visited_from_node']: 
		if nTmp[0]==R.origin: continue
		P11,P12=get_shortest_and_second(R, nTmp)
		if P12 == None or P11==None: #road is forced over e.. cannot be removed..
			C=inf
			break
		w11=sumWeights(R,P11)
		w12=sumWeights(R,P12)
		assert w12>=w11
		wstore=e[2]['weight']
		e[2]['weight']=inf #force other ways..

		P21,P22=get_shortest_and_second(R, nTmp)
		if P21==None or P22==None:
			C=inf
			break
		w21=sumWeights(R,P21)
		w22=sumWeights(R,P22)
		assert w22+eps>=w21 #closest one is the one with load
		assert w22+w21+eps>=w11+w12 #new route should be longer or equal
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



