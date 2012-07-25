if __name__=='__main__':
	import os, sys,shutil
	cmd_folder = os.path.split(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0])[0]
	print cmd_folder
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
from math import *
import time
import random
import copy

import networkx as nx
import numpy as np
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt

from graph_alg.grid import *
import graph_alg.costFunctions as cf
from graphph_alg.costFunctions import routingCost as cost
import graph_alg.graph_operations as go
from graph_alg.draw import *
from functions import getDistance
from common import get_shortest_and_second, sortRemList, remove_edge

class ProbListGen:
	"""
	a class that returns a probability list. e.g.

	for N=5, p0=0.5:

	pdf=0.5^[1,2,3,4,5]/sum(0.5^[1,2,3,4,5])=
	   =[0.5, 0.25, 0.125, 0.625. 0.3125]

	which is normalized... sum(pdf)=1

	But the list returned is the cdf (discrete one), that is [0.5, 0.75, 0.875, ...]

	It is usually used with the method getList(N) which
	returns a list of length N based on the pdf above (dependent on N)

	It may seem pretty strange to solve all this with creating a class, instead of using arbitrary lists. But I do actually like this implementation, a lot less fuss in the code where the stuff really happens.. 
	"""
	def __init__(self,p0=0.6, Nmax=15):
		self.p0=p0
		self.Nmax=Nmax
		ls=p0**(np.array(range(Nmax))+1)
		ls/=(sum(ls)+1e-10)
		assert sum(ls)<1
		assert sum(ls)>0.98
		for i in range(len(ls)):
			if i==0: continue
			ls[i]=ls[i-1]+ls[i]
		self.baseList=ls
	def getList(self, N):
		"""
		returns a list of length N with the object's distribution, normalized..
		"""
		if N>=self.Nmax:
			return self.baseList
		assert isinstance(N,int)
		a=np.array(range(N))+1
		ls=self.p0**a #pdf
		ls/=(sum(ls)+1e-10) #normalized
		assert sum(ls)<1
		assert sum(ls)>0.98
		for i in range(len(ls)): #make cdf
			if i==0: continue
			ls[i]=ls[i-1]+ls[i]
		return ls
	
def stochastic_several(R, aCap=0.20, beta=1.5, t=60, probListGen=None):
	"""
	runs simulations until we have reached the time above (seconds).
	picks the best one. Returns list of costs considered as well.
	"""
	tries=[]
	tic=time.time()
	best=None
	while time.time()-tic<t:
		seed=int(random.uniform(0,100000))
		print "seed:", seed
		random.seed(seed)
		Rtmp=copy.deepcopy(R)
		Rtmp=stochastic(Rtmp, aCap=aCap, beta=beta, probListGen=probListGen)
		if not best or best.cost>Rtmp.cost:
			best=Rtmp
		tries.append(Rtmp.cost)
		print "time left:",t-(time.time()-tic)
	return best,tries
	
	
	


def stochastic(R, ax=None, aCap=0.20, beta=1.5, anim=False, probListGen=None):
	"""
	a simplified algorithm. Still a little bit complex dock, but this is as simple as it gets I think.

	Does not add any edges, only removal. No stochastic elements are involved.

	anim=True creates a movie.

	probListGen can be given. That is a class of type ProbListGen that defines a specific
	distribution that is used by the stochastic parts.
	"""
	if probListGen==None: #use default, 0.5^i
		probListGen=ProbListGen(0.5,15)
	R.beta=beta	
	inf = 1e15
	eps=1e-9
	choiceMax=15 #we will not randomly choose something bigger than this.
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
				if not node in d['visited_from_node']:
					d['visited_from_node'].append(node) #in order to later see...

	remList=[]
	for e in R.edges(data=False): #add to remove list and calc. costs
		e_data=R.get_edge_data(*e)
		e_data['c']=cost(R, e, storeData=True)
		remList.append(e)
	remList=sortRemList(R,remList)
	choices=[0]*15 #for statistics areound the stochastics.
	while len(remList)>0: #the loop where edges are removed..

		"""
		We have a problem since the cost saved is not necessary updated to the correct value.
		Actually, the only way to update it correctly is to scan through all edges for every removal. One could think that it would be possible to only update the ones affected by the removal, i.e. the ones connected by the shortest roads. The problem is that we also need to account for the ones that would take this road if some other arbitrary road was removed. Storing that variable would be possible but very memory inefficient. It could be tried and developed further, but would only be necessary if we add edges because the below alg. works pretty good.
		
		We use the fact that the cost from removing edges can only be bigger, i.e when removing edges it only gets worse.
		
		Thus, if the updated cost is still the smallest in the list, we know that this road is cheapest to remove.
		"""
		probList=probListGen.getList(N=len(remList))
		if anim: #it's showtime..
			R.movieFlush()
		r=random.uniform(0,1)
		choice=choiceMax
		for i,p in enumerate(probList): #time to make the choice..
			if r<p:
				choice=i #usually 0.. 50% prob of that.
				break
		updated=[] #store edges that we have updated the cost for..
		done=False
		while not done: #two possibilities to break out below..
			done=True
			for i in range(choice+2): #loop over all necessary edges
				try:
					e=remList[i]
				except IndexError: #remList is too short.. we are done here...
					done=True
					break #breaks out of for-loop, not while loop
				e_data=R.get_edge_data(*e)
				if not e in updated: #saves us some calculations
					done=False #have to iterate once more.
					e_data['c']=cost(R,e,storeData=False)
					updated.append(e)
					if e_data['c']>=inf:
						remList.remove(e)
					break #otherwise we "jump over" one in the list. new for loop
				if e_data['c']>=inf:
					remList.remove(e)
				remList=sortRemList(R,remList) #sort it..
			remList=sortRemList(R,remList) #last time.. may be a double sort but who cares?
		#sorting remList and updating cost procedure is now done.
		if len(remList)==0: break #we are done
		if choice>=len(remList): #may happen if edges have been removed due to inf. cost
			choice=int(floor(random.uniform(0,len(remList)))) #take a random one..
			print "choice:", choice
		e=remList[choice]
		e_data=R.get_edge_data(*e)
		remList.remove(e)
		e_data['c']=cost(R,e,storeData=True) #in order to store the new path..
		assert e_data['c']<inf
		#following lines just to check, not used... remove later..
		if __debug__ and choice != -1 and len(remList)>choice+1: 
			e2=remList[choice+1]
			e2_data=R.get_edge_data(*e2)
			assert e_data['c']<=e2_data['c']

		#we are outside... will we go over the areaLimit if we remove e?
		if e_data['c']>eps and R.areaCover-go.singleRoadSegmentCoverage(e, R, remove=True)*R.Ainv<aCap:
			assert abs(R.areaCover-go.roadAreaCoverage(R))<eps #compare internal and actual.
			break #we are finished
		print "removes edge ",e, e_data['c'], cost(R,e)
		assert R.degree(e[0])>2 and R.degree(e[1])>2 #cost func should have given c=inf.
		remove_edge(e, R) #remove from R.
		choices[choice]+=1
	if anim:
		R.movieFlush(final=True)
	print "construction finished."
	print "road area coverage:", R.areaCover
	print "total area:", R.A
	print "choices:", choices
	R.cost=cf.totalCost(R)
	print "total cost:", R.cost
	return R
	
