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
from algorithms.stochastic import stochastic, stochastic_several, ProbListGen



###############
# This is just a bunch of Linus' scripts.. not a part of the "program/software"
#################







def testInterpolation():
	globalOrigin=596673,6728492
	areaPoly=list(5*np.array([(0,0),(48,0), (48,96), (0,96)]))
	for i in range(len(areaPoly)): areaPoly[i]=list(areaPoly[i])
	g=gr.ExtendedGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	fig=plt.figure()
	ax=fig.add_subplot(121, aspect='equal')
	print g
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

def tryP0(areaPoly=None, t=60 ):
	fig=plt.figure()
	lst=[0.5,0.6,0.7, 0.8]
	data=[]
	best=None
	globalOrigin=( 596250, 6727996)
	Rini=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	R=copy.deepcopy(Rini)
	ref=simplified_bruteForce(R)
	print "cost:", cf.totalCost(ref)
	for index,p0 in enumerate(lst):
		R=copy.deepcopy(Rini)
		R, tries=stochastic_several(R, t=t/float(len(lst)), probListGen=ProbListGen(p0,15))
		if not best or R.cost<best.cost:
			best=R
		data.append(tries)
	mi=min([min(tries) for tries in data]+[ref.cost])
	ma=max([max(tries) for tries in data]+[ref.cost])
	for index,p0 in enumerate(lst):
		ax=fig.add_subplot('%d1%d'%(len(lst), index+1))
		ax.plot(data[index], 'o')
		ax.plot([0, len(data[index])], [ref.cost, ref.cost], 'r') #reference
		ax.set_title('p0=%.1f'%p0)
		ax.set_ylabel('cost')
		ax.set_ylim(mi-10,ma+10)
		ax.set_xticklabels([])
	fig=plt.figure()
	ax=fig.add_subplot(111)
	print "best cost:", best.cost
	best.draw(ax=ax)
def best(areaPoly=None,t=60):
	"""
	time - how long we should look for a better solution
	"""
	globalOrigin= 596250, 6727996
	R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	#R=simplified_bruteForce(R,aCap=0.2, anim=True)
	R, tries=stochastic_several(R, t=t)
	fig=plt.figure()
	ax=fig.add_subplot(111)
	print "best cost:", R.cost
	R.draw(ax=ax)
	fig=plt.figure()
	ax=fig.add_subplot(111)
	ax.plot(tries, 'o')
	ax.set_ylim(0,max(tries)+100)

	
def tmp(areaPoly=None):
	globalOrigin= 596250, 6727996
	R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
	simplified_bruteForce(R,aCap=0.2, anim=False)
	c=None
	for i in range(5):
		Rt=copy.deepcopy(R)
		Rt=simplified_bruteForce(Rt,aCap=0.2, anim=False)
		if c: assert c==Rt.cost
		c=Rt.cost
	
	R=simplified_bruteForce(R,aCap=0.2, anim=False)
	#R=stochastic(R,aCap=0.2)
	fig=plt.figure()
	ax=fig.add_subplot(111, aspect='equal')
	R.draw(ax)

	print "road area coverage:", go.roadAreaCoverage(R)
	print "internal evaluation of above:", R.areaCover
	print "total area:", fun.polygon_area(areaPoly)
	print "used total area:", R.A, R.Ainv
	print "total cost:", cf.totalCost(R)

	#ax2=fig.add_subplot(224)
	#plotWorstRoad(R, ax, ax2)

def findBugs(algList):
	"""
	algorithms is a list of algorithms that should be tested
	"""
	while True:
		seed=int(random.uniform(0,1000000))
		print "seed:", seed
		random.seed(seed)
		alg=random.choice(algList)
		print "chosen algorithm:", alg
		dx=random.uniform(-700, 700)
		globalOrigin= 596250+dx, 6727996
		areaPoly=list(random.uniform(2,3)*np.array([(0,0),(48,0), (73, 105), (0,96)]))
		for i in range(len(areaPoly)): areaPoly[i]=tuple(areaPoly[i])
		R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
		R=alg(R,aCap=0.2, anim=False)
	print "road area coverage:", go.roadAreaCoverage(R)
	print "internal evaluation of above:", R.areaCover
	print "total area:", fun.polygon_area(areaPoly)
	print "used total area:", R.A, R.Ainv
	print "total cost:", cf.totalCost(R)

	
import cProfile
import time
import random
tic=time.clock()
areaPoly=list(3*np.array([(0,0),(48,0), (73, 105), (0,96)]))
for i in range(len(areaPoly)): areaPoly[i]=tuple(areaPoly[i])
#tmp(areaPoly)
#cProfile.run('''best(t=60,areaPoly=areaPoly)''')
testInterpolation()
#best(areaPoly,60*3)
#tryP0(areaPoly, t=60)
#findBugs([simplified_bruteForce, stochastic])

#testAngles()
#compareAddAndDontAdd()
#compareBruteAndSimpleBrute()
print "program took: ", time.clock()-tic, " seconds"
plt.show()
