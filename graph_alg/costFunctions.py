#import networkx as nx
from math import *
import matplotlib.pyplot as plt
import numpy as np
from grid import *
import copy
import graph_operations as go
import functions as fun

def totalCost(R):
	"""
	prototype, just to get a hint of how good it is..
	"""
	C=0
	for n in R.nodes(data=True):
		C+=R.beta*go.sumWeights(R,n[1]['shortest_path'])
		C+=go.sumWeights(R,n[1]['second_shortest'])
	return C/len(R.nodes())

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
	c=go.pathsDiff(R,e,storeData)
	assert c>=0 #should always be..
	return c/len(R.nodes())
