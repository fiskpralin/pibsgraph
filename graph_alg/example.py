"""
this is an example of how it could work.
just a script. Not a part of the "software-code"..
"""
import os, sys #insert /dev to path so we can import these modules.
if __name__=='__main__':
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import GIS.GIS as GIS
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import numpy as np
import random
import copy

from draw import draw_custom, draw_road
import graph_operations as go
import costFunctions as cf
import functions as fun
import grid as gr
import spider_grid as sg
from algorithms.bruteForce import bruteForce
from algorithms.simplified_bruteForce import simplified_bruteForce

def testInterpolation():
	globalOrigin=596673,6728492
	areaPoly=list(5*np.array([(0,0),(48,0), (48,96), (0,96)]))
	for i in range(len(areaPoly)): areaPoly[i]=list(areaPoly[i])
	g=gr.ExtendedGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	fig=plt.figure()
	ax=fig.add_subplot(121, aspect='equal')
	g.draw(ax=ax)
	#now, get the interpolation line
	from random import uniform as un
	import time
	tic=time.clock()
	for i in range(100000):
		points=[(un(100,200), un(100,200)),(un(259,400), un(250,400)) ] #points for straight line.
		x,y,z=g.getLineElevationCurve(points[0], points[1], points=50)
	print "it took ",time.clock()-tic, "seconds for 100000 interpolations with 50 points" 
	
	points=[(200, 300), (100,480)] #points for straight line.
	x,y,z=g.getLineElevationCurve(points[0], points[1], points=200)
	plt.plot(x,y, lw=3)
	ax2=fig.add_subplot(122)
	d=np.sqrt(x**2+y**2)-np.sqrt(x[0]**2+y[0]**2) #distance from start point
	ax2.set_xlabel('d')
	ax2.plot(d,z, lw=2)

def plotWorstRoad(R, ax1, ax2):
	#just for show..
	worst=None
	inf=1e15
	for edge in R.edges(data=True):
		if not worst or edge[2]['weight']>worst[2]['weight'] and edge[2]['weight']<inf:
			worst=edge
	#plot road on road-net graph.
	print "worst road, weight:", worst[2]['weight']
	from matplotlib.lines import Line2D
	l=Line2D((worst[0][0], worst[1][0]), (worst[0][1], worst[1][1]), color='r', lw=5)
	ax1.add_line(l)
	#plot elevation curve.
	x,y,z = R.getLineElevationCurve(worst[0], worst[1], points=max(5, int(fun.getDistance(worst[0], worst[1])*5)))
	x0,y0=worst[0]
	d=np.sqrt((x-x0)**2+(y-y0)**2)
	ax2.plot(d,z, lw=2)

def testAngles():
	"""
	tests the angle funciton for the roads..
	i.e. remove functions for graph
	"""
	#first, check if removing and adding gives the same result..
	a=(0,0)
	b=(20,0)
	c=(20,20)
	d=(0,20)
	e=(15,10)
	nodes=[a,b,c,d, e]
	G=gr.ExtendedGraph(areaPoly=[(-5,-5),(50,-5), (50,50), (-5,50)])
	G.add_nodes_from(nodes)
	edges=[(a,b), (b,c), (c,d), (d,a), (a,e), (b,e), (c,e), (d,e)]
	G.add_edges_from(edges)
	a=G.areaCover
	assert abs(a-go.roadAreaCoverage(G))<1e-8
	G.remove_edges_from(edges)
	assert abs(G.areaCover-0)<1e-8
	for e in edges:
		G.add_edge(e[0], e[1])
	for i in range(200): #many times
		for e in edges:
			G.remove_edge(e[0], e[1])
		for e in edges:
			G.add_edge(e[0], e[1])
	assert R.areaCover==a
	print "you passed the test.."
	G.draw(overlap=True)
def compareBruteAndSimpleBrute():
	dx=random.uniform(-700, 700)
	globalOrigin= 596250+dx, 6727996
	
	ls=np.linspace(1.4,2,5)
	t1=[]
	t2=[]
	c1=[]
	c2=[]
	el=[]

	it=1
	aCap=0.20
	for i in ls:
		areaPoly=list(i*np.array([(0,0),(48,0), (73, 105), (0,96)]))
		for itmp in range(len(areaPoly)): areaPoly[itmp]=tuple(areaPoly[itmp])
		
		R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
		c=0
		t=0
		for itmp in range(it):
			tic=time.clock()
			print R.areaCover
			assert R.areaCover>0.19

			R1=bruteForce(copy.deepcopy(R),aCap=aCap,add=False)
			R1.cost=cf.totalCost(R1)
			t+=time.clock()-tic
			c+=R1.cost
		t1.append(t/float(it))
		c1.append(c/float(it))
		el.append(R.elements)

		c=0
		t=0
		for itmp in range(it):
			tic=time.clock()
			print R.areaCover
			assert R.areaCover>0.19
			R2=simplified_bruteForce(copy.deepcopy(R),aCap=aCap)
			R2.cost=cf.totalCost(R2)
			t+=time.clock()-tic
			c+=R2.cost
		t2.append(t/float(it))
		c2.append(c/float(it))

	fig=plt.figure()
	ax1=fig.add_subplot(121)
	ax1.plot(el, t1, 'b')
	ax1.plot(el, t2, 'r')
	ax1.set_title('total time in seconds')
	plt.legend(['bruteforce','simplified bf'])

	ax2=fig.add_subplot(122)
	ax2.plot(el, c1, 'b')
	ax2.plot(el, c2, 'r')
	ax2.set_title('road cost')

	fig2=plt.figure()
	ax=fig2.add_subplot(121,aspect='equal')
	ax.set_title("bruteforce, cost: %f"%R1.cost)
	R1.draw(ax)
	ax2=fig2.add_subplot(122,aspect='equal')
	ax2.set_title("simplified bf, cost: %f"%R2.cost)	
	R2.draw(ax2)


	
def compareAddAndDontAdd():
	dx=random.uniform(-700, 700)
	globalOrigin= 596250+dx, 6727996
	
	ls=np.linspace(1.4,3,5)
	t1=[]
	t2=[]
	c1=[]
	c2=[]
	el=[]

	it=1
	aCap=0.20
	for i in ls:
		areaPoly=list(i*np.array([(0,0),(48,0), (73, 105), (0,96)]))
		for itmp in range(len(areaPoly)): areaPoly[itmp]=tuple(areaPoly[itmp])
		
		R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
		c=0
		t=0
		for itmp in range(it):
			tic=time.clock()
			print R.areaCover
			assert R.areaCover>0.19

			R1=bruteForce(copy.deepcopy(R),aCap=aCap,add=False)
			R1.cost=cf.totalCost(R1)
			t+=time.clock()-tic
			c+=R1.cost
		t1.append(t/float(it))
		c1.append(c/float(it))
		el.append(R.elements)

		c=0
		t=0
		for itmp in range(it):
			tic=time.clock()
			print R.areaCover
			assert R.areaCover>0.19
			R2=bruteForce(copy.deepcopy(R),aCap=aCap,add=True)
			t+=time.clock()-tic
			c+=cf.totalCost(R2)
		t2.append(t/float(it))
		c2.append(c/float(it))

	fig=plt.figure()
	ax1=fig.add_subplot(121)
	ax1.plot(el, t1, 'b')
	ax1.plot(el, t2, 'r')
	ax1.set_title('total time in seconds')
	plt.legend(['without add','with add'])

	ax2=fig.add_subplot(122)
	ax2.plot(el, c1, 'b')
	ax2.plot(el, c2, 'r')
	ax2.set_title('road cost')

	fig2=plt.figure()
	ax=fig2.add_subplot(121,aspect='equal')
	ax.set_title("without add, cost: %f"%R1.areaCover)
	R1.draw(ax)
	ax2=fig2.add_subplot(122,aspect='equal')
	ax2.set_title("with add, cost: %f"%R2.areaCover)	
	R2.draw(ax2)
def tmp():
	dx=random.uniform(-700, 700)
	globalOrigin= 596250+dx, 6727996
	areaPoly=list(5*np.array([(0,0),(48,0), (73, 105), (0,96)]))
	for i in range(len(areaPoly)): areaPoly[i]=tuple(areaPoly[i])
	R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	from algorithms.crazyIdea import crazyIdea
	#R=crazyIdea(areaPoly=areaPoly)
	#R=gr.TriGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	#R=sg.SpiderGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	R=simplified_bruteForce(R,aCap=0.2,add=False)
	print "road area coverage:", go.roadAreaCoverage(R)
	print "internal evaluation of above:", R.areaCover
	print "total area:", fun.polygon_area(areaPoly)
	print "used total area:", R.A, R.Ainv
	fig=plt.figure()
	ax=fig.add_subplot(111, aspect='equal')
	R.draw(ax)
	#ax2=fig.add_subplot(224)
	#plotWorstRoad(R, ax, ax2)
	

	
import cProfile
import time
import random
random.seed(1)
tic=time.clock()
#cProfile.run('''tmp()''')
tmp()

#testAngles()
#compareAddAndDontAdd()
#compareBruteAndSimpleBrute()
print "simulation took: ", time.clock()-tic, " seconds"
plt.show()
