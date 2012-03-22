import networkx as nx
import costFunc as cf
import numpy as np
from math import sin, cos, tan, pi

def update_after_mod(e,R):
	"""
	updates graph after edge was removed or added. Assumes that routingcost function has stored correct data before.
	"""
	print "updates some data"
	for nTmp in e[2]['visited_from_node']: #each node that visited the removed edge
		P1=nTmp[1]['second_shortest']+nTmp[1]['shortest_path']
		P2=nTmp[1]['new_second_shortest']+nTmp[1]['new_shortest_path']
		#ax=testRoads(R, nTmp[1]['new_shortest_path'], nTmp[1]['new_second_shortest'], ax) #used for debugging
		for path, diff in (P1, -1), (P2, 1):
			last=path[0]
			for nTmp2 in path[1:]: #loop over the edges in path
				if nTmp2==last: continue #overlap between roads, skip
				try:
					d=R.get_edge_data(*(last, nTmp2))
					d['visits']=d['visits']+diff
					if diff==-1 and nTmp in d['visited_from_node']:
						d['visited_from_node'].remove(nTmp)
					elif not nTmp in d['visited_from_node']:
						d['visited_from_node'].append(nTmp)
				except ValueError:
					print "remove failed", d['visited_from_node']
					print d['visited_from_node'].count(nTmp),nTmp
					raise Exception('sads')
				except:
					if not ((last==e[0] and nTmp2==e[1]) or (last==e[1] and nTmp2==e[0])): #if not the removed edge
						print  (last, nTmp2), e[0:2], diff
						from draw import *
						ax=draw_custom(R, edge_visits=True)
						plot_coverage(R,ax)
						plt.show()
						raise Exception('tries to modify edge that does not exist, something is wrong')
				last=nTmp2
		nTmp[1]['shortest_path']=nTmp[1]['new_shortest_path']
		nTmp[1]['second_shortest']=nTmp[1]['new_second_shortest']
def remove_edge(e, R):
	"""removes edge e from R and updates related statistics"""
	dA=cf.singleRoadSegmentCoverage(e, R, remove=True)
	R.remove_edge(e[0], e[1])
	R.graph['areaCover']-=dA*R.graph['Ainv']
	update_after_mod(e,R)
def add_edge(e, R):
	"""adds e to R and updates statistics."""
	dA=cf.singleRoadSegmentCoverage(e, R, add=True)
	R.add_edges_from([tuple(e)])
	a=R.get_edge_data(e[0], e[1])
	R.graph['areaCover']+=dA*R.graph['Ainv']
	e[2]['c']=cf.routingCost(R,e,storeData=True)
	#update_after_mod(e,R)
def get_angle(e1,e2):
	"""
	returns the angle between roads. angle<=pi, i.e. the smallest one is always chosen.

	Clarification: straight road means angle=pi
	"""
	#first, determine which node that is intersecting
	node=e1[0]
	if node in [e2[0], e2[1]]:
		r1=np.array(e1[1])-np.array(node)
	else:
		node=e1[1] #try the other one...
		if not node in [e2[0], e2[1]]: raise Exception('edges are not connected')
		r1=np.array(e1[0])-np.array(node)
	if node==e2[0]:
		r2=np.array(e2[1])-np.array(node)
	else: #node is e2[1]
		r2=np.array(e2[0])-np.array(node)
	#so, we have two vectors stationed in the origin... the rest is just linear algebra..			
	norm=np.linalg.norm
	d=np.vdot(r1,r2) #this is abs(r1)*abs(r2)*cos(theta)
	arg=d/(norm(r1)*norm(r2))
	if arg>1: #this happens very rarely, some numerical fault.
		arg=1
	elif arg<-1:
		arg=-1
	th=np.arccos(arg)

	th=abs(th)
	if th>pi: th=2*pi-th
	return th
def overLapA(e1,e2, R):
	"""
	computes the overlapping area of two road segments. Does not handle the dictionary routines at all.

	OBS:
	-this function returns the whole overlap-area. In order to get a correct compensation, half of this area has to be used since it is counted twice.
	-This function gives the overlap, it doesn't cont for the fact of "negative overlap", i.e. the area that should be a part of the road but is not counted.
	"""
	e1=e1[0:2]
	e2=e2[0:2]
	#first, find out if this calculation has been performed before.
	try:
		overlap=R.graph['overlap']
	except: #create dictionary
		R.graph['overlap']={} 
		overlap=R.graph['overlap']
	#The key structure ((from1, to1), (from2, to2)) gives us four alternatives. Simply try them all.
	#room for optimization...
	keylist=[((e1[0], e1[1]), (e2[0], e2[1])),
			 ((e1[1], e1[0]),(e2[1], e2[0])),
			 ((e2[1], e2[0]),(e1[1], e1[0])),
			 ((e2[0], e2[1]), (e1[0], e1[1]))]
	for key in keylist:
		try:
			a=overlap[key]
			return a
		except KeyError:
			pass #try the next one in keylist
	#we did not find it... calculate instead. Only done once.
	angle=get_angle(e1,e2) 	#get angle between the road segments..
	eps=1e-8
	if angle<eps:
		raise Exception('angle=0 between roads... incident? something must be wrong')
	if abs(angle-pi)<eps: return 0 #straight line-.. no overlaps
	alpha=pi-angle #this is the angle that matters here...
	#the following variables look like "mumbo-jumbo" It's all trigonometry but should be documented somewhere...
	d=0.5*R.graph['w']
	x=2*d*sin(alpha*0.5)
	y=d*cos(alpha*0.5)
	z=0.5*x/tan((pi-alpha)/2)
	a=x*0.5*(y+z)
	overlap[(e1,e2)]=a #save so we don't have to calculate next time
	return a
if __name__=='__main__':
	#try the overlap thing.
	G=nx.Graph()
	nodes=[(0,0), (1,0), (1,1)]
	for node in nodes:
		G.add_node(node)
	G.add_edge((0,0), (1,0), weight=1)
	G.add_edge((1,0), (1,1), weight=1)
	G.graph['w']=1
	angle=get_angle(G.edges()[0], G.edges()[1]) #just test of the angle..
	if angle-np.pi/2.>0.0001: raise Exception('either get_angle doesnt work or we are creating graph in the wrong way')
	print overLapA(G.edges()[0], G.edges()[1], G)
	
	
	
