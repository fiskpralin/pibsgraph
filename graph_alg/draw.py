import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib.patches import Circle
from matplotlib.lines import Line2D  


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
	ax.set_xlim(*tuple(G.graph['lim'][0:2]))
	ax.set_ylim(*tuple(G.graph['lim'][2:4]))
	return ax
def middle(p1,p2):
	"""returns the point in the middle"""
	p1=np.array(p1)
	p2=np.array(p2)
	d=p2-p1
	return list(p1+0.5*d)
def plot_coverage(G=None, ax=None):
	if not G or not ax: raise Exception()
	for e in G.edges(data=True):
		p1=np.array(e[0])
		p2=np.array(e[1])
		d=p1-p2
		circles=int(np.sqrt(np.dot(d,d))/6.)#one every 6m
		for i in range(circles):
			p=p2+d*(i+1)/(circles+1)
			c=Circle(p, radius=G.graph['C'], alpha=470, color='#666677')
			ax.add_patch(c)
	return ax
def draw_road(p, ax, color='c'):
	last=None
	for node in p:
		if last:
			ax.plot((last[0], node[0]), (last[1], node[1]), color, linewidth=3)
		last=node
