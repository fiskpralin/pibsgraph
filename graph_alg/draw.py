import numpy as np
import os
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.lines import Line2D
from matplotlib.path import Path
import matplotlib.patches as patches
import functions as fun
from mpl_toolkits.mplot3d import axes3d, Axes3D # <-- NOTE!
import matplotlib.image as mpimg
from matplotlib.colors import LinearSegmentedColormap
from matplotlib import cm

def draw_custom(G=None, ax=None, edge_visits=False, cost=False, weight=False,road_color='k', road_width=1, poly=False):
	assert G != None
	if edge_visits or cost or weight:
		debug_mode=True #means that we do not plot bends that nicely... provides debug info
	else:
		debug_mode=False
	if not ax:
		fig=plt.figure()
		ax=fig.add_subplot(111, aspect='equal')
		ax.set_axis_bgcolor('#ffbb55')
		ax.set_xlabel('x')
		ax.set_ylabel('y')
	if not debug_mode: plotBends(G=G, ax=ax, road_color=road_color, road_width=road_width)
	for e in G.edges(data=True):
		x=e[0][0], e[1][0]
		y=e[0][1], e[1][1]
		pos=middle([x[0],y[0]], [x[1],y[1]]) #used for edge text..
		if weight:
			ax.text(pos[0],pos[1],' '+'w:%.0f'%e[2]['weight'])
		if cost:
			if e[2]['c']>1e10:
				ax.text(pos[0],pos[1],' '+r'$\infty$')
			else:
				ax.text(pos[0],pos[1]-2,' '+'%.0f'%e[2]['c'])
		if not debug_mode and e[2]['plotted']: continue #already have a nice bending curve
		if edge_visits:
			l = Line2D(x,y, color=road_color, linewidth=road_width+np.log(1+len(e[2]['visited_from_node']))/(np.log(len(G)*0.5)*0.2))
			ax.text(pos[0],pos[1]+2,' '+'%d'%len(e[2]['visited_from_node']))
		else:
			l = Line2D(x,y, color=road_color, linewidth=road_width)
		ax.add_line(l)
	ax.set_xlim(*tuple(G.lim[0:2]))
	ax.set_ylim(*tuple(G.lim[2:4]))


	#plot the polygon
	if poly:
		p = Polygon(G.areaPoly,closed=True, color='none', ec='k',lw=3, ls='solid')
		ax.add_patch(p)
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
		   	l = Line2D(x,y, color=road_color, linewidth=road_width)
			#now, update 'plotted' thing.
			ax.add_line(l)
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
	"""
	plots coverage from the road. Shows overlaps and spots missed.
	"""
	if not G or not ax: raise Exception('need ax and G to plot coverage')
	points=15
	angles=np.linspace(0, np.pi, points) #for half circle
	for e in G.edges(data=False):
		p1=np.array(e[0])
		p2=np.array(e[1])
		d=p1-p2
		x=e[0][0], e[1][0]
		y=e[0][1], e[1][1]
		w=G.C*2
		nodes=[]
		r,dir=fun.getCylindrical(e[1], origin=e[0])
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

def plotSurface(x=None,y=None,z=None):
	"""
	plots the surface of a terrain grid given x,y,z
	"""
	if z==None or y==None or x==None: raise Exception('x,y,z needs to be given.')
	if len(z)!=len(y) or len(y)!=len(x): raise Exception('lengths of vectors needs to be identical')
	elmax=200000000 #will not plot more than this number of elements
	fig = plt.figure()
	ax = Axes3D(fig)
	ax.grid(True)
	x,y,z=np.array(x),np.array(y),np.array(z)
	el=len(x[0])*len(y)
	step=np.ceil(float(el)/elmax)
	print "step:", step, " elements:", el, "elements used:", el/step**2
	ax.plot_surface(x,y,z, rstride=step, cstride=step, cmap=cm.autumn)
	ax.set_zlim3d(50, 105)
	ax.set_title('terrain')
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_zlabel('z [m]')
	
	fig = plt.figure()
	ax2 = Axes3D(fig)
	ax2.contour(x,y,z, zdir='z')
	ax2.set_zlim3d(50, 105)
	plt.colorbar()
	return ax


####
# The following functions all do plots of raster GIS data.
####


def plot2DContour(x=None,y=None,z=None, ax=None, w=1, colorscheme=cm.Greens):
	"""
	plots the contours of z  in xy-plane
	"""
	if z==None or y==None or x==None: raise Exception('x,y,z needs to be given.')
	if len(z)!=len(y) or len(y)!=len(x): raise Exception('lengths of vectors needs to be identical')
	x,y,z=np.array(x),np.array(y),np.array(z) #if not already
	if not ax:
		fig = plt.figure()
		ax = fig.add_subplot(111, aspect='equal')
	minZ=min([min(ztmp) for ztmp in z])
	maxZ=max([max(ztmp) for ztmp in z])
	levels=np.linspace(minZ,maxZ, 25) #15 lines
	c=ax.contour(x,y,z, zdir='z', linewidths=w, levels=levels,cmap=colorscheme)
	cb=plt.colorbar(c)
	cb.lines.set_linewidth(10)
	return ax

def plot3DContour(x=None,y=None,z=None, ax=None):
	"""
	plots the contours of z  in xy-plane
	"""
	if z==None or y==None or x==None: raise Exception('x,y,z needs to be given.')
	if len(z)!=len(y) or len(y)!=len(x): raise Exception('lengths of vectors needs to be identical')
	x,y,z=np.array(x),np.array(y),np.array(z) #if not already
	if not ax:
		fig = plt.figure()
		ax =Axes3D(fig)
	ax.contour(x,y,z, zdir='z')
	return ax

def plotBackground(areaPoly=None, ax=None, globalOrigin=None):
	"""
	plots the background from the tiff-file given. In the future, maybe data is taken from
	google maps or something similar?

	#local coordinates in areaPoly, with origin in origin
	origin is given in sweref99 coordinates, not local.
	"""	
	if not areaPoly: raise Exception('need area polygon in order to plot background')
	if not ax:
		fig=plt.figure()
		ax=fig.add_subplot(111, aspect='equal')
	figCorner=(595000, 6725000) #for this specific area, not general		
	if not globalOrigin:
		globalOrigin=figCorner
	#now, let our global origin be in the origin..
	w=5000 #for this specific image..
	h=5000
	figorigin=np.array(figCorner)-np.array(globalOrigin)
	limits=[figorigin[0], figorigin[0]+w, figorigin[1], figorigin[1]+h]
	folder=os.path.dirname(os.path.abspath(__file__))
	folder=os.path.join(folder, 'GIS')
	map=LinearSegmentedColormap.from_list('lightgray', ['#666666', '#C5C5C5']) #our own cmap
	bg=mpimg.imread(os.path.join(folder,'672_59_11.tif'))
	im = plt.imshow(bg, cmap=map,origin='lower', extent=limits)
	
	#now, adjust to our polygon..
	limits=list(fun.polygonLim(areaPoly))
	w=max(10, max(limits)/10.)
	limits[0]-=w #to get some space to polygon
	limits[1]+=w
	limits[2]-=w
	limits[3]+=w
	ax.axis(limits)
	return ax


def draw_road(p, ax, color='c'):
	assert ax
	last=None
	for node in p:
		if last:
			ax.plot((last[0], node[0]), (last[1], node[1]), color, linewidth=3)
		last=node
	return ax
	


