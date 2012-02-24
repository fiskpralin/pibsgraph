import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from math import *

from terrain.terrain import Terrain
from machines.forwarder.forw import Forwarder
from machines.harvester.harv import Harvester
from sim.tools import SimSeries, SimExtend, globalVar
import functions as fun
import plot
from road import Road
import sim.tools as tools
import graph_alg as ga


###############################
# Forwarder sim
###############################

def setupDefaultRoadNet(G):
	"""setups the default net, with two loops"""
	G.roadNet=nx.Graph()
	rN=G.roadNet
	nodes=[(12,0), (36,0), (60,0), (84,0), (84,70), (60,70), (36,70), (12,70)]
	rN.add_nodes_from(nodes)
	edges=[((12,0),(36,0)), ((36,0),(60,0)), ((60,0),(84,0)), ((84,0),(84,70)), ((84,70),(60,70)), ((60,70),(60,0)), ((36,0),(36,70)),((36,70), (12,70)), ((12,70),(12,0)), ((60,70),(36,70))]
	rN.add_edges_from(edges)
	for e in rN.edges(data=True):
		e[2]['weight']=fun.getDistance(e[0], e[1])
	for e in rN.edges(data=True):
		e[2]['r']=Road(endPoints=e[0:2])
def setUpTerrain(G=None):
	"""setups the default terrain with roads"""
	if not G.areaPoly: G.areaPoly=[(0,-12), (96,-12), (96,82),(0,82) ]
	t=Terrain(G=G,generate=True, dens=0.1)
	if not G.roadNet:
		setupDefaultRoadNet(G)
		G.roadNet.graph['origin']=(12,0)
	return t
		
def simRandomRoad(G=None,vis=True, anim=False):
	"""
	Generates a random road, and simulates the forwarder on it.
	"""
	if not G: G=globalVar()
	if not G.areaPoly: G.areaPoly=[(0.0,0.0), (100.0, 0.0), (100.0,100.0), (180, 180), (100, 170),(-100,100.0)]
	if not G.roadNet: G.roadNet=ga.construct.makeRoadGraph(L=24,ulim=(0,1),angle=0.2, areaPoly=G.areaPoly, grid='spider')
	for e in G.roadNet.edges(data=True):
		e[2]['r']=Road(endPoints=e[0:2]) #road instance, that can be used in simulations.
	ForwarderSim(G, vis, anim)
	
class ForwarderSim(SimExtend):
	"""
	class for a single simulation with a 1a or 2a plantmachine
	"""
	def __init__(self, G=None, vis=True, anim=False):
		SimExtend.__init__(self,G, vis=vis, anim=anim, animDelay=5) #does some common stuff, i.e seed
		if not self.G.terrain:
			self.G.terrain=setUpTerrain(self.G)
		h=Harvester(self.G) #does all the harvester stuffi n the constructor.
		self.trees=0 #total number of picked up trees
		self.m=Forwarder(name="forwy", sim=self, G=self.G)
		self.activate(self.m,self.m.run())
		self.simulate(until=1000000)
		if self.p:
			self.p.terminate()
			#postprocessing
			plt.show()
		print "simulation is done."
	def plotMoni(self, ax, number):
		"""
		defines the monitors that should be plotted.
		"""
		if number==1: #plot driver activity
			moni=self.m.totalTreeMoni
			if len(moni)>0:
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([-0.1, self.now(), -0.1, (max(moni.yseries())+1)])
				ax.grid(True)
				ax.set_xlabel('time(s)')
				ax.set_ylabel('trees picked up')
		elif number==2: #plot trees planted
			moni=self.m.distMoni
			if len(moni)>0:
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([-0.1, self.now(), -0.1, (max(moni.yseries())+1)])
				ax.grid(True)
				ax.set_xlabel('time(s)')
				ax.set_ylabel('dist. traveled (m)')
		elif number==3:
			#create a new axis..
			plt.delaxes(ax)
			ax=self.p.fig.add_subplot(326, aspect='equal', axisbg='#A2CD5A')
			lim=self.m.pos[0]-15, self.m.pos[0]+15, self.m.pos[1]-15, self.m.pos[1]+15
			self.p.update_world(ax,lim)
		return ax
				
		
