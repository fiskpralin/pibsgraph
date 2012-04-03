"""
this is an example of how it could work.
just a script
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

from draw import draw_custom
import costFunc as cf
import functions as func
import grid as gr
import spider_grid as sg
import alg 

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
	ax.add_line(l)
	#plot elevation curve.
	x,y,z = R.getLineElevationCurve(worst[0], worst[1], points=max(5, int(func.getDistance(worst[0], worst[1])/2)))
	d=np.sqrt(x**2+y**2)
	ax2.plot(d,z, lw=2)


globalOrigin= 596352, 6727996
areaPoly=list(2*np.array([(0,0),(48,0), (48,96), (0,96)]))
for i in range(len(areaPoly)): areaPoly[i]=tuple(areaPoly[i])
R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
#R=gr.TriGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
#R=sg.SpiderGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
alg.cycleRoad(R,aCap=0.21)
print "road area coverage:", cf.roadAreaCoverage(R)
print "total area:", func.polygon_area(areaPoly)
print "used total area:", R.A, R.Ainv
fig=plt.figure()
ax=fig.add_subplot(121, aspect='equal')
R.draw(ax)
ax2=fig.add_subplot(122)
plotWorstRoad(R, ax, ax2)

plt.show()
