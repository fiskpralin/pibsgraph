import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib.patches import Circle, Polygon
from matplotlib.lines import Line2D

import functions as fun

def draw_custom(G=None, ax=None, edge_visits=False, cost=False, road_color='k', road_width=1):
	if not G: raise Exception()
	if not ax:
		fig=plt.figure()
		ax=fig.add_subplot(111, aspect='equal')
		ax.set_axis_bgcolor('#ffbb55')
		ax.set_xlabel('x')
		ax.set_ylabel('y')
	for n in G.nodes():
		ax.add_patch(Circle(n, radius=1, facecolor='k', alpha=1))
	for e in G.edges(data=True):
		x=e[0][0], e[1][0]
		y=e[0][1], e[1][1]
		if edge_visits:
			l = Line2D(x,y, color=road_color, linewidth=road_width+np.log(e[2]['visits'])/(np.log(len(G))*0.2))
			pos=middle([x[0],y[0]], [x[1],y[1]])
			if cost: ax.text(pos[0],pos[1],' '+'%.0f'%e[2]['c'])
		else:
			l = Line2D(x,y, color=road_color, linewidth=road_width)
		ax.add_line(l)
	ax.set_xlim(*tuple(G.lim[0:2]))
	ax.set_ylim(*tuple(G.lim[2:4]))


	#plot the polygon
	poly = Polygon(G.areaPoly,closed=True, color='none', ec='k',lw=3, ls='solid')
	ax.add_patch(poly)
	return ax
def middle(p1,p2):
	"""returns the point in the middle"""
	p1=np.array(p1)
	p2=np.array(p2)
	d=p2-p1
	return list(p1+0.5*d)
def plot_coverage(G=None, ax=None):
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
			print loc
			p=fun.getCartesian(loc, origin=e[1], direction=dir, fromLocalCart=True)
			nodes.append(p)


		nodes.append(fun.getCartesian([-w/2.,l], origin=e[0], direction=dir, fromLocalCart=True))
		nodes.append(fun.getCartesian([-w/2.,0], origin=e[0], direction=dir, fromLocalCart=True))
		#fix the other half circle
		for ang in angles:
			loc=fun.getCartesian([w/2., ang]) #get local cartesian from cyl.
			p=fun.getCartesian(loc, origin=e[0], direction=dir-np.pi, fromLocalCart=True)
			nodes.append(p)
		pol = Polygon(nodes, alpha=150, color='#666677', edgecolor='none')
		ax.add_patch(pol)
	return ax
def draw_road(p, ax, color='c'):
	last=None
	for node in p:
		if last:
			ax.plot((last[0], node[0]), (last[1], node[1]), color, linewidth=3)
		last=nodex
	
