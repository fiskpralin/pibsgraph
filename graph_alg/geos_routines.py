import networkx as nx
from construct import *
from string import *
import os
import random

def makeGEOSinput(G):
	"""
	creates a file of the .tsp type.
	"""
	#startstring="NAME: nynas\nTYPE: TSP\nCOMMENT: temporary file \nDIMENSION: %d\nEDGE_WEIGHT_TYPE: EUC_2D\nNODE_COORD_SECTION"%(len(G.nodes()))
	filename='steinerTemp.tsp'
	f=open(filename, 'w')
	#f.write(startstring+'\n')
	i=0
	nodes=G.nodes()
	nodes=sorted(nodes, key=lambda node: random.uniform(0,1))#distance to origin (assume 0,0)
	for n in nodes:
		i+=1
		#f.write('%d %.1f %.1f\n'%(i, n[0], n[1]))
		f.write('%.1f \t %.1f\n'%(n[0], n[1]))
	f.write('EOF')
	f.close()
	return f




if __name__=='__main__':
	L=24
	origin=(0.0,0.0)
	#areaPoly=[(0.0,0.0), (100.0, 0.0), (100.0,100.0), (180, 180), (100, 170),(-100,100.0)]
	Ls=200.0
	areaPoly=[(0.0,0.0), (Ls, 0.0), (Ls,Ls),(0.0,Ls)]
	R=makeRoadGraph(L=L,ulim=(0,0),origin=origin, angle=0, areaCap=1,areaPoly=areaPoly, diagonals=False)
	f=makeGEOSinput(R)
