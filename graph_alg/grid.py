#!/usr/bin/env python
from math import *
import os, sys,shutil

import networkx as nx
import numpy as np
from scipy.interpolate import RectBivariateSpline 
from matplotlib.patches import Polygon
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt

import GIS.GIS as GIS
import graph_operations as go
if __name__=='__main__':
	import os, sys #insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import functions as fun
import collision as col
import draw
from weightFunctions import weightFunctionFirst, normalizedPitchDist


class ExtendedGraph(nx.Graph):
	"""
	intended as an extension of networkx graph with some features that we want.

	Grids inherit from this class, with basically just a different constructor.

	Modifies networkX methods for adding and removing edges/nodes so that we can do some more refined statistics for this.

	
	"""
	def __init__(self, origin=None, globalOrigin=None,areaPoly=None,L=None, C=None, gridtype=None):
		if not areaPoly:
			raise Exception('areaPoly must be given.')
		if not origin:
			xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
			self.origin=xmin, ymin
		else:
			self.origin=origin
		if not globalOrigin:
			globalOrigin=(596120, 6727530) #sweref99..located on map.. nice position.. use as default.
		assert C!=None
		assert L!=None
		self.L=L
		self.C=C
		self.areaPoly=areaPoly
		xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
		side=10
		self.lim=np.array([xmin-0.5*side,xmax+0.5*side, ymin-0.5*side, ymax+0.5*side])
		self.A=fun.polygon_area(areaPoly)
		self.Ainv=1./self.A #used a lot.. faster to just compute this once.
		self.globalOrigin=globalOrigin
		self.type=gridtype
		x,y,z=GIS.readTerrain(globalOrigin=globalOrigin , areaPoly=areaPoly)
		#get a list of how x and y varies.. strictly ascending
		xlist=x[:,0] #just the variations, not the 2D-matrix
		ylist=y[0,:]
		self.interpol=RectBivariateSpline(xlist, ylist, z) #used pretty much everytime we need the height of a specific point. Implemented in fortran and very fast. Supposedly one can add smoothing factor s~75 for approximating splines instead.

		nx.Graph.__init__(self)
		self.t_x=x
		self.t_y=y
		self.t_z=z
		self.roadWidth=4 #width of road
		self.overlap={} #will later be filled. A speedup thing
		self.weightFunction=normalizedPitchDist #reference to exter
		self.areaCover=go.roadAreaCoverage(self)
		self.moviefig=None #may be used for movies later
		self.cmdfolder=os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
		self.aPInnerCircleM=None #used later for polygon calculations

	def getSimplePitch(self,p1,p2,points=20):
		"""
		Gives the pitch along a line from point p1 to point p2.
		output:
		pitch - a list with the pitch in every point on the line
		Not tested, and not called 
		"""
		pitch=[]
		x=np.linspace(p1[0], p2[0], points)
		y=np.linspace(p1[1], p2[1], points)
		z=self.interpol.ev(x,y)
		length=fun.getDistance(p1,p2)
		piInv=1/pi
		for ent in range(len(z)):
			if ent==0:
				pitch.append(180*piInv*atan2(z[ent+1]-z[ent],length/float(points))) #forward difference
			elif ent==len(z)-1:
				pitch.append(180*piInv*atan2(z[ent+1]-z[ent],length/float(points))) #backward difference
				break
			else:
				pitch.append(180*piInv*atan2(z[ent+1]-z[ent-1],2*length/float(points))) #central difference
		if len(pitch)!=len(z): raise Exception('Something wrong with the dimensions of the pitch')
		return pitch

	def getRoll(self,p1,p2,points=20,style='naive'):
		"""
		Provided choice of style is 'naive' or 'weighted', this method outputs the roll along a
		road specified by the start point p1 and end point p2.
		Not tested, and not called
		"""
		alpha=atan2((p2[0]-p1[0]),(p2[1]-p1[1]))
		length=fun.getDistance(p1,p2)
		piInv=1/pi
		roll=[]
		p11=fun.getCartesian([-self.roadwidth*0.5,0], direction=pi*0.5-alpha, origin=p1,fromLocalCart=True)
		p12=fun.getCartesian([self.roadwidth*0.5,0], direction=pi*0.5-alpha, origin=p1,fromLocalCart=True)
		p21=fun.getCartesian([-self.roadwidth*0.5,0], direction=pi*0.5-alpha, origin=p2,fromLocalCart=True)
		p22=fun.getCartesian([self.roadwidth*0.5,0], direction=pi*0.5-alpha, origin=p2,fromLocalCart=True)
		x1=np.linspace(p11[0], p21[0], points)
		x2=np.linspace(p12[0], p22[0], points)
		y1=np.linspace(p11[1], p21[1], points)
		y2=np.linspace(p12[1], p22[1], points)
		z1=self.interpol.ev(x1,y1)
		z2=self.interpol.ev(x2,y2)
		if style=='naive':#This is the most naive method, called naiveroll in intro_interpol.py
			for ent in range(len(z1)):
				roll.append(180*piInv*atan2((z2[ent]-z1[ent]),self.roadwidth))
		elif style=='weighted':#This method is originally called GISroll or GIScopy-method in intro_interpol.py
			for ent in range(len(z1)):
				if ent==0:
					roll.append(180*piInv*atan2(((2*z2[ent]+z2[ent+1])-(2*z1[ent]+z1[ent+1])),6*self.roadwidth*0.5))
				elif ent==len(z1)-1:
					roll.append(180*piInv*atan2(((z2[ent-1]+2*z2[ent])-(z1[ent-1]+2*z1[ent])),6*self.roadwidth*0.5))
					break
				else:
					roll.append(180*piInv*atan2(((z2[ent-1]+2*z2[ent]+z2[ent+1])-(z1[ent-1]+2*z1[ent]+z1[ent+1])),8*self.roadwidth*0.5))
		else: raise Exception('getRoll need a correct style to be supplied at call')
		return roll

		
	def getLineElevationCurve(self,p1,p2, points=10):
		"""
		interpolates from the terrain data and returns line coordinates.
		p1-start point
		p2-end point

		output:
		x - array of x-values along line
		y - array of y-values along line
		"""
		x=np.linspace(p1[0], p2[0], points)
		y=np.linspace(p1[1], p2[1], points)
		return x, y, self.interpol.ev(x,y) #evaluate interpolation at above points.

	def getPointAltitude(self,p):
		"""
		interpolates from the terrain data and returns point coordinates.

		not tested yet
		"""
		x=np.array(p[0])
		y=np.array(p[1])
		return list(self.interpol.ev(x,y)) #evaluate interpolation at above points.

	def edgeWeightCalc(self,p1,p2):
		"""
		calculates the weight of the edge between p1 and p2.

		So far this is just a simple example, but it should be expanded.

		reason for using self.weightFunction is that several externa functions can be tested..
		"""
		d=fun.getDistance(p1,p2)
		return d #remove! just temporary...
		x,y,z=self.getLineElevationCurve(p1,p2, points=max(5, int(d/2))) #every 2 m at least..
		w=self.weightFunction(x,y,z)
		return w
	
	def remove_node(self,n):
		"""
		like the standard one, but takes away edge first so some data is stored that we need
		"""
		for neig in self.neighbors(n):
			self.remove_edge(n,neig)
		super(ExtendedGraph, self).remove_node(n=n)
		
	def remove_nodes_from(self,nodes):
		"""
		we need to update data, so we only use the above one.
		"""
		for n in nodes:
			self.remove_node(n)
		
	def remove_edge(self, e1, e2):
		"""
		removes edge e from R and updates related statistics
		If more functionality is needed for your road-algorithm, override or write new function
		"""
		super(ExtendedGraph,self).remove_edge(u=e1,v=e2)
		dA=go.singleRoadSegmentCoverage((e1,e2), self, remove=True)
		self.areaCover-=dA*self.Ainv

	def remove_edges_from(self, ebunch):
		"""
		removes edge e from R and updates related statistics
		If more functionality is needed for your road-algorithm, override or write new function
		"""
		for e in ebunch: #one at a time because of overlap-calculations..
			dA=go.singleRoadSegmentCoverage((e[0],e[1]), self, remove=True)
			self.areaCover-=dA*self.Ainv
			super(ExtendedGraph,self).remove_edges_from(ebunch=[e])


	def add_edge(self, e1, e2, attr_dict=None, **kwargs):
		"""
		adds e to edges and updates statistics.
		"""
		e=(e1,e2, kwargs)
		self.add_edges_from([e])

	def add_edges_from(self, ebunch, attr_dict=None, **kwargs):
		"""
		same as above..but for many
		"""
		for e in ebunch: #because of area overlap we need to take one at a time
			super(ExtendedGraph,self).add_edges_from(ebunch=[e], attr_dict=attr_dict, attr=kwargs)
			dA=go.singleRoadSegmentCoverage((e[0],e[1]), self, add=True)
			self.areaCover+=dA*self.Ainv
		
	def edges_from_path_gen(self, path, data=False):
		"""
		returns a generator that steps through the edges between the nodes..
		
		e.g.:
		path=[(1,1),(2,2)(3,3)]
		edges=[((1,1),(2,2)), ((2,2),(3,3))]
		
		very handy..
		if data=True, data for each edge is given as well as edge[2].
		Check if the edges exist if data=True

		It is a class method in order to be able to provide data
		"""
		if path and len(path) != 0:
			assert len(path) != 1 #one node does not make an edge...
			last=None
			for node in path:
				if last: #not first time..
					if data:
						d=self.get_edge_data(*(last,node))
						yield (last, node, d) #with data. We also know that it exists.
					else:
						yield (last, node) #the edge.. we do not really check if it exists but..
				last=node

	def movieFlush(self,final=False, fps=3):
		"""
		used to create movie. Call for every frame. To finally produce movie, give final=True
		First time, first=True has to be set.

		Creates a folder animtemp that is not deleted, so that the frames can be used.

		However, the content is deleted next time the movie-function is used.

		The fps input value is only used when final=True

		Movies can usually be made through the algorithm functions. If not, this method provides a nice way to do it yourself.
		"""
		folder=os.path.join(self.cmdfolder, 'animtemp')
		if not self.moviefig: #first one..
			self.moviefig=plt.figure()
			self._plot_i=0
			if os.path.exists(folder):
				shutil.rmtree(folder)
			os.makedirs(folder)
		if not os.path.exists(folder):
			raise Exception('Need to give first=True first time movieFlush is called.')
		if final: #no more recording, this is the last one..
			name=os.path.join(self.cmdfolder,'animation.avi')
			os.system("mencoder 'mf://%s/_tmp*.png' -mf type=png:fps=%d -ovc lavc -lavcopts vcodec=wmv2 -oac copy -o %s"%(folder,fps, name)) #only works if mencoder is installed and on linux. Maybe mac as well.. not sure.
		else:
			ax=self.moviefig.add_subplot('111')
			ax=self.draw(ax=ax)
			self.moviefig.savefig(os.path.join(folder,'_tmp%06d.png'%round(self._plot_i)))
			self.moviefig.clear() #saves some memory and we can re-draw next time.
			self._plot_i+=1

	def inside(self,pos):
		"""
		are we inside the polygon? We don't use colission detection directly since we can do
		some time saving tests
		"""
		if self.aPInnerCircleM==None: #created yet?
			self.aPInnerCircleM, self.aPInnerCircleRadius=fun.getPolygonInnerCircle(self.areaPoly)
		if fun.insideCircle(pos, self.aPInnerCircleM, self.aPInnerCircleRadius):
			return True #much faster than below
		return col.pointInPolygon(pos,self.areaPoly)


	def draw(self, ax=None, overlap=False, weight=False, cost=False, edge_visits=False, background=True, contour=True):
		"""
		does all the plotting. Should be able to determine if we have terrain data etc.
		"""
		if not ax: #add a new one.. 
			fig=plt.figure()
			ax=fig.add_subplot(111,aspect='equal')
		if background:
			ax=draw.plotBackground(globalOrigin=self.globalOrigin , areaPoly=self.areaPoly, ax=ax)
		if contour:
			ax=draw.plot2DContour(self.t_x,self.t_y,self.t_z,ax, w=2)
		draw.draw_custom(G=self, ax=ax, cost=cost,weight=weight,edge_visits=edge_visits, road_color='#01243B', road_width=4, poly=False)
		#bug in matplotlib always plots patch in back.. do line instead
		vertices=self.areaPoly+[self.areaPoly[0]] #closed
		x=[v[0] for v in vertices]
		y=[v[1] for v in vertices]
		l=Line2D(x,y, color='k', linewidth=4)
		ax.add_line(l)
		if overlap:
			draw.plot_coverage(self,ax, color='r')
		return ax




class SqGridGraph(ExtendedGraph):
	"""
	A square grid graph with dx=dy=L.
	xyRatio =0 creates a square area. xyRatio>1 creates an "x>y" rectangle.
	angle is how the grid is turned in radians. 0=default
	Strategy for angle: setup the "base line" by turning the coordinate system by angle. Do a while loop where y is incremented and decremented until we are out of borders.
	"""
	def __init__(self,L=24, xyRatio=1,origin=None, globalOrigin=None,areaPoly=None, diagonals=False, angle=None, shift=(0,0)):
		C=L/2.0
		ExtendedGraph.__init__(self, origin=origin, globalOrigin=globalOrigin,areaPoly=areaPoly, L=L, C=C, gridtype='sqGridGraph')
		xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
		origin=self.origin
		
		if angle ==None: #find the longest edge, and use its angle
			angle, shift=self.getAngleFromLongestEdge()
			print "got the angle.", angle*360/(2*3.14), shift
			while angle>pi/2. or angle<0: #if it is.. most often does not enter loop
				if angle<0:
					angle+=pi/2.
				else:
					angle-=pi/2.
		dx=L
		dy=L
		cart=fun.getCartesian
		digits=3 #we round of to this in order to avoid numerical errors. 1mm is enough :)
		"""The strategy is to first cover an extra large area, and then take away the nodes that are outside.
		for a square area, the maximum "x-distance" is sqrt(2) times the side. This corresponds to angle pi/4 or
		5pi/4. The strategy is to always use this maximum length and then take away the ones outisde the area."""
		d=1/sqrt(2)+0.001
		#direction is by definition not bigger that 90 degrees:
		ym=sqrt((xmax-xmin)**2+(ymax-ymin)**2)
		xlmin=-ceil(sin(pi/2.-angle)*(ymax-ymin)/L)*L
		xlmax=ceil(sin(angle)*(xmax-xmin)/L)*L
		xl=np.arange(shift[0]+xlmin,xlmax, dx, dtype=np.float)
		yl=np.arange(shift[1],ym, dy, dtype=np.float)
		for xloc in xl:
			for yloc in yl:
				if yloc==0 or angle==0: sl=[1]
				x, y=tuple(cart((xloc,yloc), origin=origin, direction=angle, fromLocalCart=True))
				x,y=round(x,digits), round(y,digits)
				self.add_node((x, y))
				#neigbor 'backwards' in y-dir
				neig=tuple(cart((xloc,(yloc)-dy), origin=origin, direction=angle, fromLocalCart=True))
				neig=round(neig[0],digits), round(neig[1],digits)
				self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y), neig), visits=0, visited_from_node=[], c=0)
				if diagonals and xloc != 0:
					neig=tuple(cart((xloc-dx,(yloc-dy)), origin=origin, direction=angle,fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y), neig), visits=0, visited_from_node=[],c=0)
					neig=tuple(cart((xloc+dx,(yloc-dy)), origin=origin, direction=angle, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y), neig),visits=0, visited_from_node=[],c=0)
				neig=tuple(cart((xloc-dx,yloc), origin=origin, direction=angle, fromLocalCart=True))
				neig=round(neig[0],digits), round(neig[1],digits)
				self.add_edge((x,y),neig, weight=self.edgeWeightCalc((x,y), neig), visits=0, visited_from_node=[],c=0)
		for node in self.nodes():
			if not self.inside(node):
				self.remove_node(node)
		rem=True
		while rem:
			rem=False
			for n in self.nodes(): #pretty ugly, but a must..
				if self.degree(n)<=1:
					rem=True
					self.remove_node(n)
					break
		if not self.origin in self.nodes():
			shortest=None
			short_dist=1e10
			for n in self.nodes():
				d=fun.getDistance(n, self.origin)
				if d<short_dist:
					short_dist=d
					shortest=n
			self.origin=shortest
		elements=len(self.nodes())
		self.elements=elements
		self.density=elements/self.A

	def getAngleFromLongestEdge(self, areaPoly=None, origin=None):
		"""
		finds the angle of the longest edge in relation to the x-axis.
		if there is a draw, the edge closest to the origin wins.
		
		This function is only usable for konvex polygons, but works for concave as well.

		the square distance is used since it's faster to compute

		the shift thing only works if the origin is in the "lower left" corner.
		"""
		if not origin:
			origin=self.origin
		if not areaPoly:
			areaPoly=self.areaPoly
		last=areaPoly[-1]
		longest=None
		dmax=0
		for node in areaPoly:
			d2=fun.getDistanceSq(node, last)
			if d2==dmax: #look for distance to origin
				dtmp=min([fun.getDistanceSq(node, origin), fun.getDistanceSq(last, origin)])
				dtmp2=min([fun.getDistanceSq(node, longest[0]), fun.getDistanceSq(last, longest[1])])
				if dtmp<dtmp2:
					longest = (node,last)
					dmax=d2
			elif d2>dmax:
				longest = (node,last)
				dmax=d2
			last=node
		#now, calculate the distance to this line.
		#we need to make a line, not a ray, in order to get the extension as well.
		infRay=np.array(longest)
		infRay=infRay+1e5*(infRay-infRay[1])+1e5*(infRay-infRay[0]) #infinite extension of longest
		pTmp, t=col.closestLinePoint(origin, infRay, True)
		assert t!=1 and t!=0
		d=fun.getDistance(pTmp,origin)
		assert d>=0
		#now, we know that d+shift=n*L+C..where n is an integer
		n=round((d-self.C)/float(self.L))
		shift=self.L*n+self.C-d
		assert abs(shift)<=self.L
		angle=fun.angleToXAxis(longest)
		assert angle>=0
		if 0<angle<=pi/2: #shift x negative
			shift=(-shift,0)
		elif pi/2<angle<=pi: #shift y negative
			shift=(0,-shift)
		elif pi<angle<=3*pi/2.0: #shift x positive
			shift=(shift,0)
		else:
			shift=(0, -shift)
		return angle, shift

	
class TriGridGraph(ExtendedGraph):
	"""
	A triangular grid. Extends from ExtendedGraph
	"""
	def __init__(self,L=None, xyRatio=1, origin=None, globalOrigin=None,angle=None, areaPoly=None):
		"""The strategy is to first cover an extra large area, and then take away the nodes that are outside.
		for a square area, the maximum "x-distance" is sqrt(2) times the side. This corresponds to angle pi/4 or
		5pi/4. The strategy is to always use this maximum length and then take away the ones outisde the area.
		This is an easy but inefficient algorithm, there is certainly room for speed up if required.
		But when coding this, most of the time is spent somewhere else so it doesn't matter really to optimize this part.

		The square grid has been updated with a better algorithm for this. It might be a good idea to copy this algorithm to this part, but it is not done now.

		Another improvement could be to optimize the inside() function. It currently does unnecessary many point in polygon tests, which is time consuming for polygons with many nodes.
		"""
		C=12
		digits=3 #used later..
		if L==None:
			L=C/(sin(1/3.0*pi)*0.5)
		ExtendedGraph.__init__(self, origin=origin, globalOrigin=globalOrigin,areaPoly=areaPoly, L=L, C=C,gridtype='triGridGraph')
		#C=L/2. #preference questions, this does not span entirely all of space but is a good compromise
		self.L=L
		self.C=C
		dx=L
		dy=L*round(sqrt(3)/2., digits)
		cart=fun.getCartesian
		#xl=np.arange(0,Nx*dx, dx, dtype



		xmin,xmax,ymin,ymax=fun.polygonLim(areaPoly)
		x1=np.arange(xmin,ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float)
		x2=np.arange(xmin-dx/2., ceil(sqrt(xmax**2+ymax**2)), dx, dtype=np.float)
		yl=np.arange(ymin,ceil(sqrt(xmax**2+ymax**2)), dy, dtype=np.float) #hypothenuse in another way
		if not angle: angle=0
		#G=nx.Graph( L=L, type='sqGridGraph', C=C)
		self.lim=np.array([xmin-0.5*L,xmax+0.5*L, ymin-0.5*L, ymax+0.5*L])
		direction=angle+pi/2.

		for yloc in yl:
			if (round(yloc/dy))%2==0:
				xl=x1
			else:
				xl=x2
			for index,xloc in enumerate(xl):
				x,y=tuple(cart((xloc,yloc), origin=(0,0), direction=direction, fromLocalCart=True))
				x,y=round(x,digits), round(y,digits)
				self.add_node((x, y))
				if y != 0:
					neig=tuple(cart([xloc-dx/2., round(yloc-dy,digits)], origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y),neig), visits=0, visited_from_node=[],c=0)
					neig=tuple(cart([xloc+dx/2., yloc-dy,digits], origin=(0,0), direction=direction, fromLocalCart=True))
					neig=round(neig[0],digits), round(neig[1],digits)
					self.add_edge((x,y), neig, weight=self.edgeWeightCalc((x,y),neig), visits=0, visited_from_node=[],c=0)
				self.add_edge((x,y),tuple([round(x-dx,digits), round(y,digits)]), weight=self.edgeWeightCalc((x,y),tuple([round(x-dx,digits), round(y,digits)])), visits=0, visited_from_node=[],c=0)
		rem=True
		for n in self.nodes():
			if not self.inside(n):
				self.remove_node(n)
		while rem:
			rem=False
			for n in self.nodes(): #pretty ugly, but a must..
				if self.degree(n)<=1:
					rem=True
					self.remove_node(n)
					break
		self.overlap={} #will later be filled.
		self.roadWidth=4
		self.L=L
		if not self.origin in self.nodes():
			shortest=None
			short_dist=1e10
			for n in self.nodes():
				d=fun.getDistance(n, self.origin)
				if d<short_dist:
					short_dist=d
					shortest=n
			self.origin=shortest
		elements=len(self.nodes())
		self.elements=elements
		self.density=elements/self.A
		self.density=elements/self.A



