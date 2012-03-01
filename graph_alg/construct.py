import time
import random
import copy
from math import *

import networkx as nx
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt

import grid as gr
import spider_grid as sg
import costFunc as cf
import graph_operations as go
import alg
from draw import *


def makeRoadGraph(L=24, origin=(0,0), grid='square', ulim=(0,0),areaPoly=None,angle=0, beta=1.25, areaCap=0.20,diagonals=False):
	"""
	constructs a road graph.
	"""
	if grid=='square':
		G=gr.sqGridGraph(L, umin=ulim[0], umax=ulim[1], xyRatio=1, origin=origin,angle=angle, diagonals=diagonals, areaPoly=areaPoly)
	elif grid =='tri' or grid=='triangular':
		G=gr.triGridGraph(L, umin=ulim[0], umax=ulim[1],origin=origin, xyRatio=1, angle=angle, areaPoly=areaPoly)
	elif grid=='spider' or grid =='Spider': 
		G=sg.SpiderGrid(L, umin=ulim[0], origin=origin,umax=ulim[1], angle=angle, areaPoly=areaPoly)
	else:
		raise Exception('grid is not supported', grid)
	if not origin in G.nodes():
		shortest=None
		short_dist=1e10
		for n in G.nodes():
			d=sqrt((n[0]-origin[0])**2+(n[1]-origin[1])**2)
			if d<short_dist:
				short_dist=d
				shortest=n
		origin=shortest
	G.graph['origin']=origin
	G.graph['beta']=beta
	G.graph['cd']=1
	G.graph['cr']=1
	G.graph['areaCap']=areaCap
	R=copy.deepcopy(G)
	R.graph['w']=4
	R.graph['areaCover']=cf.roadAreaCoverage(R)
	alg.cycleRoad(G,R,aCap=True)
	return R

