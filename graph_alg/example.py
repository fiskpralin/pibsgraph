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
from construct import makeRoadGraph
from draw import draw_custom


#remove later
import grid as gr


globalOrigin=(596120, 6727530) #sweref99..located on map.. nice position..
areaPoly=[(0,0), (1000,100), (1000,600), (700,1000), (300,1000)]
x,y,z=GIS.readTerrain(globalOrigin=(596120, 6727530) , areaPoly=areaPoly)
ax=GIS.plotBackground(areaPoly, globalOrigin=globalOrigin)
ax=GIS.plot2DContour(x,y,z,ax)
pol=Polygon(areaPoly, closed=True, color='none', ec='k',lw=3, ls='solid')
ax.add_patch(pol)
#R=makeRoadGraph(grid='spider', areaPoly=areaPoly)
R=gr.sqGridGraph(areaPoly=areaPoly)
print "--------------"
print "nodes:", len(R.nodes()), areaPoly
draw_custom(R, ax=ax, road_color='b', road_width=3)
plt.show()
