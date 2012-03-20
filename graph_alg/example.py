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


#remove later
import grid as gr


globalOrigin=(596120, 6727530) #sweref99..located on map.. nice position..
#areaPoly=[(0,0), (200,50), (300,150), (50,100), (0,150)]
areaPoly=list(np.array([(0,0),(48,0), (48,48), (0,48)]))
for i in range(len(areaPoly)): areaPoly[i]=list(areaPoly[i])
x,y,z=GIS.readTerrain(globalOrigin=(596120, 6727530) , areaPoly=areaPoly)
ax=GIS.plotBackground(areaPoly, globalOrigin=globalOrigin)
ax=GIS.plot2DContour(x,y,z,ax)
pol=Polygon(areaPoly, closed=True, color='none', ec='k',lw=3, ls='solid')
ax.add_patch(pol)
R=makeRoadGraph(grid='tri', areaPoly=areaPoly)
#R=gr.sqGridGraph(areaPoly=areaPoly)
print "road area coverage:", cf.roadAreaCoverage(R)
print "total area:", func.polygon_area(areaPoly)
print "used total area:", R.graph['A'], R.graph['Ainv']
draw_custom(R, ax=ax, road_color='b', road_width=3)
plt.show()
