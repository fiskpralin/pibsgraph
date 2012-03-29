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

from construct import makeRoadGraph
from draw import draw_custom
import costFunc as cf
import functions as func
import grid as gr
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




globalOrigin=596673,6728492
areaPoly=list(3*np.array([(0,0),(48,0), (48,96), (0,96)]))
for i in range(len(areaPoly)): areaPoly[i]=tuple(areaPoly[i])
R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
alg.cycleRoad(R,aCap=0.20)
print "road area coverage:", cf.roadAreaCoverage(R)
print "total area:", func.polygon_area(areaPoly)
print "used total area:", R.graph['A'], R.graph['Ainv']
R.draw()


plt.show()
