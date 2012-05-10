import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib.patches import Circle, Polygon
from matplotlib.lines import Line2D
from matplotlib.path import Path
import matplotlib.patches as patches
import functions as fun

def draw_custom(G=None, ax=None, edge_visits=False, cost=False, weight=False,road_color='k', road_width=1):
	if not G: raise Exception()
	if not ax:
		fig=plt.figure()
		ax=fig.add_subplot(111, aspect='equal')
		ax.set_axis_bgcolor('#ffbb55')
		ax.set_xlabel('x')
		ax.set_ylabel('y')
	for n in G.nodes():
		ax.add_patch(Circle(n, radius=1, facecolor='k', alpha=1))
	plotBends(G=G, ax=ax, road_color=road_color, road_width=road_width)
	for e in G.edges(data=True):
		x=e[0][0], e[1][0]
		y=e[0][1], e[1][1]
		pos=middle([x[0],y[0]], [x[1],y[1]]) #used for edge text..
		if weight:
			ax.text(pos[0],pos[1],' '+'%.0f'%e[2]['weight'])
		if cost: ax.text(pos[0],pos[1],' '+'%.0f'%e[2]['c'])
		if e[2]['plotted']: continue #already have a nice bending curve
		if edge_visits:
			l = Line2D(x,y, color=road_color, linewidth=road_width+np.log(e[2]['visits'])/(np.log(len(G))*0.2))
		else:
			l = Line2D(x,y, color=road_color, linewidth=road_width)
		ax.add_line(l)
	ax.set_xlim(*tuple(G.lim[0:2]))
	ax.set_ylim(*tuple(G.lim[2:4]))


	#plot the polygon
	poly = Polygon(G.areaPoly,closed=True, color='none', ec='k',lw=3, ls='solid')
	ax.add_patch(poly)
	return ax
def plotBends(G=None, ax=None, road_color='k', road_width=1):
	"""
	identifies nodes with only two neighbors. IF we make a bend at that node, plots a bezier curve, i.e. a not that sharp curve.

	Sets edge[2]['plotted'] to True if plotted out for the two neighboring edges
	"""
	edges=G.edges(data=True)
	for edge in edges:
		edge[2]['plotted']=False #change later where it applies.
	if len(edges)>1000: return ax #the beziers curve takes time to plot. Too big system.

	for n in G.nodes(data=False):
		if nodeEdgesBendable(G,n):
			neighbors=G.neighbors(n)
			p1=tuple(fun.getPointBetween(neighbors[0], n))
			p2=tuple(fun.getPointBetween(neighbors[1], n))
			verts= [neighbors[0], p1, n, p2, neighbors[1]]
			codes = [Path.MOVETO, Path.LINETO,Path.CURVE3,Path.CURVE3,Path.LINETO]
			#now, if any of the neighbors have degre 2 AND a bend, we should only plot half
			#of the way there.
			if nodeEdgesBendable(G,neighbors[0]):
				verts[0]=p1 #no line to neighbor, only half way
			if nodeEdgesBendable(G,neighbors[1]):
				verts[-1]=p2
			path = Path(verts, codes)
			p=patches.PathPatch(path, facecolor='none', edgecolor=road_color, lw=road_width)
			#ax.add_patch(patches.PathPatch(path, facecolor='none', edgecolor=road_color, lw=road_width))

			#because of bug in matplotlib, p always shows in the back of the plot.
			#Make a line instead. 
			vertices=p.get_verts()
			x=[v[0] for v in vertices]
			y=[v[1] for v in vertices]
			l=Line2D(x,y, color=road_color, linewidth=road_width)
			ax.add_line(l)
			#now, update 'plotted' thing.
			for neigh in neighbors:
				d=G.get_edge_data(n, neigh)
				d['plotted']=True

def nodeEdgesBendable(G,n):
	"""
	Determines if we can make a bend at node n.

	This applies if degree(n)==2 and we don't have a straight line.

	Only works for ~90 degree corners
	"""
	if G.degree(n)==2:
		#find out if we have a bend.
		neighbors=G.neighbors(n)
		ray1=n, neighbors[0]
		ray2=n, neighbors[1]
		th=fun.getAngle(ray1,ray2)
		if abs(th-np.pi)>0.01 and abs(th-np.pi/2.)<0.1:
			#not a straight line and corner angle ~90 degrees
			return True
	return False

def middle(p1,p2):
	"""returns the point in the middle"""
	return fun.getPointBetween(p1,p2)

def plot_coverage(G=None, ax=None, color='#666677'):
	if not G or not ax: raise Exception()
	points=15
	angles=np.linspace(0, np.pi, points) #for half circle
	for e in G.edges(data=False):
		"""p1=np.array(e[0])
		p2=np.array(e[1])
		d=p1-p2
		circles=int(np.sqrt(np.dot(d,d))/6.)#one every 6m
		for i in range(circles):
			p=p2+d*(i+1)/(circles+1)
			c=Circle(p, radius=G.C, alpha=470, color='#666677')
			ax.add_patch(c)"""
		x=e[0][0], e[1][0]
		y=e[0][1], e[1][1]
		try:
			w=G.C*2
		except:
			w=24
		nodes=[]
		dir=fun.angleToXAxis(e)-np.pi/2.
		l=fun.getDistance(e[0], e[1])
		nodes.append(fun.getCartesian([w/2.,0], origin=e[0], direction=dir, fromLocalCart=True))
		nodes.append(fun.getCartesian([w/2.,l], origin=e[0], direction=dir, fromLocalCart=True))
		#now, fix end half circle.
		for ang in angles:
			loc=fun.getCartesian([w/2., ang]) #get local cartesian from cyl.
			p=fun.getCartesian(loc, origin=e[1], direction=dir, fromLocalCart=True)
			nodes.append(p)


		nodes.append(fun.getCartesian([-w/2.,l], origin=e[0], direction=dir, fromLocalCart=True))
		nodes.append(fun.getCartesian([-w/2.,0], origin=e[0], direction=dir, fromLocalCart=True))
		#fix the other half circle
		for ang in angles:
			loc=fun.getCartesian([w/2., ang]) #get local cartesian from cyl.
			p=fun.getCartesian(loc, origin=e[0], direction=dir-np.pi, fromLocalCart=True)
			nodes.append(p)
		pol = Polygon(nodes, alpha=150, color=color, edgecolor='none')
		ax.add_patch(pol)
	return ax

def draw_road(p, ax, color='c'):
	last=None
	for node in p:
		if last:
			ax.plot((last[0], node[0]), (last[1], node[1]), color, linewidth=3)
		last=node
	
