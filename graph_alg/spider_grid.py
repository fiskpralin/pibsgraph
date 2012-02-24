if __name__=='__main__':
	import os, sys #insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
		
import collision as col
import copy

from grid import *
from draw import *
from functions import *


class Line():
	lines=0
	def __init__(self, p1, p2, angle, order):
		self.p1=tuple(p1)
		self.p2=tuple(p2)
		self.length=getDistance(p1,p2)
		self.ray=[p1,p2]
		self.angle=angle
		self.gridPoints=[self.p1,self.p2] #if you insert one, make sure that p2 remains last..
		Line.lines+=1
		self.no=Line.lines #personal identifier, we know that this is a unique number
		self.order=order

def SpiderGrid(L=24, umin=0, umax=0, diagonals=False, angle=None, areaPoly=None, origin=None, thMin=pi/7.):
	"""
	A grid that looks like a spider's net
	"""
	C=L/2.
	longest=L*1.4 #longest allowed distance between two nodes on a line.
	eqL=L/6. #length where two points should be merged to one,m which is done in some cases
	if not areaPoly: #use default
		areaPoly=[(0,0), (300,0), (200,300), (-100, 200)]
		if origin: raise Exception('if origin is given, so must areaPoly...')
		origin=(0,0)
		if not origin in areaPoly: raise Exception('only origin at borderpoints supported right now ')
	else:
		if not origin: raise Exception('origin has to be defined for the SpiderGrid if area polygon is given')
		if not origin in areaPoly:
			raise Exception('so far, SpiderGrid only supports origins on one of the border points')
	for i in range(len(areaPoly)): #find origin
		if areaPoly[i]==origin:
			origInd=i
	xnorm=[origin, (origin[0]+1, origin[1])] #xaxis, used for angle calc.
	ray1=[areaPoly[origInd], areaPoly[origInd-1]]
	ray2=[areaPoly[origInd], areaPoly[origInd+1]]
	baseL=Line(ray1[0], ray1[1], getAngle(ray1, xnorm), order=1)
	baseR=Line(ray2[0], ray2[1], getAngle(ray2, xnorm), order=1)
	#so, we have the bases.. check if left and right is correct
	if baseL.angle<baseR.angle:
		tmp=baseL
		baseL=baseR
		baseR=tmp
	angle=baseL.angle-asin(L*0.5/getDistance(baseL.ray[0], baseL.ray[1]))
	p2=findIntersection(list(baseL.p1), angle, areaPoly)
	cyl=getCylindrical(p2, origin=origin) #take it further away from the border
	p2=getCartesian([cyl[0]-C, cyl[1]], origin=origin)
	baseL=Line(p1=baseL.p1, p2=p2, angle=angle, order=baseL.order)
	
	angle=baseR.angle+asin(L*0.5/getDistance(baseR.ray[0], baseR.ray[1]))
	p2=findIntersection(list(baseR.p1), angle, areaPoly)
	cyl=getCylindrical(p2, origin=origin) #take it further away from the border
	p2=getCartesian([cyl[0]-C, cyl[1]], origin=origin)
	baseR=Line(p1=baseR.p1, p2=p2, angle=angle, order=baseL.order)
	lines=[baseR, baseL]
	th=getAngle(baseL.ray, baseR.ray)
	dth=th
	order=1
	while True: #create lines, base roads
		dth=dth/2.
		if dth<thMin: break
		newList=copy.deepcopy(lines)
		#order+=1
		for i in range(len(lines)): #iterate through, from right to left
			line=lines[i]
			if line.no != baseL.no:
				th=line.angle+(lines[i+1].angle-line.angle)/2.
				point=findIntersection(origin, th, areaPoly)
				cyl=getCylindrical(point, origin=origin) #take it further away from the border
				point=getCartesian([cyl[0]-C, cyl[1]], origin=origin)
				lnew=Line(origin, point, th, order=order)
				for ind,ltmp in enumerate(newList):
					if ltmp.no==line.no:
						newList.insert(ind+1, lnew)
						break
		lines=newList
	#the base roads are done.. now let's find the smaller lines.
	#strategy: insert a line between two lines as long as asin(0.5L/length(line))>L
	while True:
		added=False
		newList=copy.deepcopy(lines)
		order+=1
		for i in range(len(lines)): #iterate through, from right to left
			line=lines[i]
			if line.no != baseL.no:
				leftBuddy=lines[i+1] #the one "to the left"
				rightBuddy=line
				#in the future, taking terrain into consideration, the angles to left and right buddy wont be the same. But now they are.
				dth=(leftBuddy.angle-rightBuddy.angle)/2. #the angle between..
				th=rightBuddy.angle+dth #angle in rel. to xaxis
				point=findIntersection(origin, th, areaPoly) #point at other side of areaPoly
				#now, make it C distance away from border.
				cyl=getCylindrical(point, origin=origin)
				point=getCartesian([cyl[0]-C, cyl[1]], origin=origin)
				d=getDistance(origin, point)
				if d/tan(dth)>L:
					#this means that it will be reasonable to place a grid point on line. Create line!
					y=0.5*L*(1+1/sin(dth))#/(sin(dth)**2)
					p1=getCartesian([0, y], fromLocalCart=True, origin=origin, direction=th) #where it all starts..
					if getDistance(p1, origin)>=d or getDistance(p1,point)<C:
						continue #don't make line, it has a "negative length" or is too short..
					if col.pointInPolygon(p1, areaPoly):
						lnew=Line(p1, point, th, order=order)
						for ind,ltmp in enumerate(newList):
							if ltmp.no==line.no:
								added=True
								newList.insert(ind+1, lnew)
		if not added: break #no more lines to add.
		lines=newList
	#make grid of lines.
	G=nx.Graph( L=L, type='sqGridGraph', C=C)

	el=0 #will later be filled.
	G.graph['L']=L
	G.graph['overlap']={} #will later be filled.
	#make a road segments to the ones "to the left". If road that ist drawn to is of higher order, that is the last one from this line.
	for index, line in enumerate(lines): #make line to the ones on the "left"
		if line==baseL: break
		occured=[] #a list of difference orders that have occured.. pretty complex.
		#an order 1 line should no e.g. have lines to all order 4 lines to the left, only one.
		for left in lines[index+1:]: # the one to the left of "line"
			#identify the point, p2. line between should be orthogonal to "line" or "left", depending on order.
			cont=False #continue...
			for o in occured:
				if o<=left.order: cont=True #strange procedure, but can't use "continue" here due to above for.
			if cont: continue #we jump over some lines..think about it and you'll udnerstand
			occured.append(left.order)
			if left.order>line.order: #make ray orthogonal to line
				p1=left.p1
				th2=line.angle-pi/2.
				pTmp=tuple(getCartesian([0, 10000], origin=p1, direction=th2, fromLocalCart=True))
				a, p2=col.linesIntersect(np.array([p1,pTmp]), np.array(line.ray), getPoint=True)
				if not a:
					p2=line.p2
				p2=tuple(p2)
				if not p2 in line.gridPoints:
					line.gridPoints.append(p2)
			else: #make it orthogonal to left
				p1=line.p1
				th2=left.angle+pi/2.
				pTmp=tuple(getCartesian([0, 10000], origin=p1, direction=th2, fromLocalCart=True))
				a, p2=col.linesIntersect(np.array([p1,pTmp]), np.array(left.ray), getPoint=True)
				if not a:
					p2=left.p2
				p2=tuple(p2)
				if not p2 in left.gridPoints:
					left.gridPoints.append(p2)
			if not a:
				pass #raise Exception('something wrong, did not find neighbor point')
			if G.has_node(p1):
				G.add_node(p1)
				el+=1
			if not p2 in G.nodes():
				G.add_node(p2)
				el+=1
			G.add_edge(p1,p2, weight=getDistance(p1,p2), visits=0, visited_from_node=[], c=0)
			if left.order <= line.order: break
	#we have the pattern. Add edges to neighbors in not that dens regions
	for index,line in enumerate(lines): 	#end points. Usually between two endpoints but not always.
		if line.no==baseL.no: break #this is the end
		leftBuddy=lines[index+1]
		closest=leftBuddy.gridPoints[0]
		closDist=getDistance(closest, line.p2)
		for pTmp in leftBuddy.gridPoints[1:]:
			d=getDistance(pTmp, line.p2)
			if d<closDist:
				closDist=d
				closest=pTmp
		G.add_edge(line.p2, closest, weight=getDistance(line.p2,leftBuddy.p2), visits=0, visited_from_node=[], c=0)
	lOrdered=copy.copy(lines) #not deepcopy, line instances are the same
	lOrdered=sorted(lOrdered, key=lambda line: line.order)
	for line in lOrdered:
		#if line.no==baseL.no: continue
		#now, identify the potential neighbors first.
		candidatesL=[l for l in lines if l.angle>line.angle]
		candidatesL=sorted(candidatesL, key=lambda line: line.order)
		candidatesR=[l for l in lines if l.angle<line.angle]
		candidatesR=sorted(candidatesR, key=lambda line: line.order)
		lst=candidatesL+candidatesR
		for l in lst: #find the real neighbors
			for l2 in lst:
				if l2.order==l.order:
					if l in candidatesL and l2.angle>line.angle and l2.angle<l.angle:
						candidatesL.remove(l) #never neighbor.
					elif l in candidatesR and l2.angle<line.angle and l2.angle>l.angle:
						candidatesR.remove(l) #never neighbor.
		line.gridPoints=sorted(line.gridPoints, key=lambda point: -getDistance(point, origin)) #closest first
		last=line.gridPoints[0]
		for pTmp in copy.copy(line.gridPoints[1:]):
			d=getDistance(last,pTmp)
			if d>longest:
				nPoints=int(ceil(d/longest))-1
				last=pTmp
				l=d/(nPoints+1) #distance between points
				for i in range(nPoints):
					p=tuple(getCartesian([0, (i+1)*l], origin=pTmp, direction=line.angle, fromLocalCart=True))
					candidatesL=sorted(candidatesL, key=lambda l: abs(l.angle-line.angle))
					candidatesR=sorted(candidatesR, key=lambda l: abs(l.angle-line.angle))
					leftRay=np.array([p, getCartesian([0,100], origin=p, direction=line.angle+pi/2., fromLocalCart=True)])
					rightRay=np.array([p, getCartesian([0,100], origin=p, direction=line.angle-pi/2., fromLocalCart=True)])
					pL=None
					pR=None
					for cand in candidatesL:
						a,p2=col.linesIntersect(np.array(cand.ray), leftRay, getPoint=True)
						if a: #cand:s are ordered so that this is "the one"
							for pT in cand.gridPoints:
								if getDistance(list(pT), p2)<eqL: #2 meters.. same
									p2=pT
									break
							pL=tuple(p2)
							if not pL in cand.gridPoints: cand.gridPoints.append(pL)
							break
					for cand in candidatesR:
						a,p2=col.linesIntersect(np.array(cand.ray), rightRay, getPoint=True)
						if a: #cand:s are ordered so that this is "the one"
							for pT in cand.gridPoints:
								if getDistance(list(pT), p2)<eqL: #2 meters.. same
									p2=pT
									break
							pR=tuple(p2)
							if not pR in cand.gridPoints: cand.gridPoints.append(pR)
							break
					#now, if both pL and pR, create a straight line.
					if pL and pR: #new p!
						a,p=col.linesIntersect(np.array(line.ray), np.array([pL,pR]), getPoint=True)
						if not a: raise Exception('expected to find p here..')
						p=tuple(p)
					if pL:
						G.add_edge(pL,p,weight=getDistance(pL,p), visits=0, visited_from_node=[], c=0)
					if pR:
						G.add_edge(pR,p, weight=getDistance(pR,p), visits=0, visited_from_node=[], c=0)
					if not pL and not pR:
						G.add_node(p, visit)
					line.gridPoints.append(tuple(p))
						
			last=pTmp
				

	
	for line in lines: #add edges on the line
		if len(line.gridPoints)<=1: continue
		line.gridPoints=sorted(line.gridPoints, key=lambda point: -getDistance(point, origin))
		last=line.gridPoints[0]
		for node in line.gridPoints[1:]:
			G.add_edge(last, node,weight=getDistance(last, node), visits=0, visited_from_node=[], c=0)
			last=node
	A=getArea(G)
	G.graph['elements']=el
	G.graph['A']=A
	G.graph['Ainv']=1./G.graph['A']
	G.graph['density']=el/G.graph['A']
	G.graph['areaPoly']=areaPoly
	lim=polygonLim(areaPoly)
	G.graph['lim']=np.array(lim)
	#graph should be done by now.
	return G
def findIntersection(origin, th, areaPoly):
	"""
	returns the intersection between the ray from origin with angle th and areaPoly. Assumes convex polygon.
	"""
	
	inf=1e4
	#find intersection with areaPoly
	p1=getCartesian([0, inf], origin=origin, direction=th, fromLocalCart=True)
	p2=getCartesian([0, 0.1], origin=origin, direction=th, fromLocalCart=True) #should be inside polygon
	ray=np.array([p1,p2])
	last=areaPoly[-1]
	point=None
	for p in areaPoly:
		borderRay=np.array([last,p])
		int, pInt=col.linesIntersect(borderRay, ray, getPoint=True) 
		if int:
			point=pInt
			break #we have found intersection point
		last=p
	if point==None:
		print col.linesIntersect(np.array([[0,200],[200,200]]), ray), ray
		raise Exception('line has no intersection with polygon area')
	return tuple(point)
if __name__=='__main__':
	areaPoly=[(0.0,0.0), (100.0, 0.0), (100.0,100.0), (180, 180), (100, 170),(-100,100.0)]
	G=SpiderGrid(areaPoly=areaPoly, origin=(0.0, 0.0))
	areaPoly=G.graph['areaPoly']
	import matplotlib as mpl
	import matplotlib.pyplot as plt
	from matplotlib.patches import Polygon
	fig=plt.figure()
	ax=fig.add_subplot(121, aspect='equal')
	#plot area
	poly = Polygon(areaPoly,closed=True, color='#ffbb55', ec='k',lw=3, ls='solid')
	ax.add_patch(poly)
	draw_custom(G, ax)
	plot_coverage(G, ax)
	lim=polygonLim(areaPoly)
	ax.set_xlim(lim[0]-10, lim[1]+10)
	ax.set_ylim(lim[2]-10, lim[3]+10)


	#plot lines
	lim=polygonLim(areaPoly)


	#for comparison
	ax=fig.add_subplot(122, aspect='equal')
	poly = Polygon(areaPoly,closed=True, color='#ffbb55', ec='k',lw=3, ls='solid')
	ax.add_patch(poly)
	G=sqGridGraph(areaPoly=areaPoly)
	draw_custom(G=G,ax=ax)
	plot_coverage(G=G, ax=ax)
	ax.set_xlim(lim[0]-10, lim[1]+10)
	ax.set_ylim(lim[2]-10, lim[3]+10)
	plt.show()

