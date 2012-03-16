#!/usr/bin/env python
import os, sys #insert /dev to path so we can import these modules.
if __name__=='__main__':

	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if cmd_folder not in sys.path:
		sys.path.insert(0, cmd_folder)
from math import *
from matplotlib.patches import Circle
import collision as col
from functions import getDistance, getCartesian, polygon_area
import functions as fun #the way it should be.
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
import time
import numpy as np
import random
from scipy.stats import pareto

#from obstacle import Obstacle
from stump import Stump
from tree import Tree
from root import Root
from boulder import Boulder
from pile import Pile

class Terrain():
	"""The main terrain class.
	If global variable object G is given, it is assumed to have the variables:'
		xlim: [min,max]
		ylim: [min, max]
		plantMinDist - the minimum distance between plants. If seedlings are not implemented you can skip this.
	areaPol - a polygon that defines  the stand. Points are expected to be counterclockwise"""
	def __init__(self,G=None, generate=False, dens=1, dbh_mean=0.5, dbh_std=0.1):
		if not G: raise Exception('by design, you need G for terrain to work. (globalVar(), see sim.tools.py)')
		self.G=G
		self.trees=[]
		self.stumps=[]
		self.roots=[]
		self.holes=[]
		self.piles=[]
		self.obstacles=[] #should hold all the above, except boulders.
		self.stumpFile='undefined'
		self.treeFile='undefined'
		self.stumpsPerH=0
		self.boulderFreq=0
		self.meanBoulderV=0
		#the following stuff are only used when the stand files are used. should maybe be separated.
		if not G.areaPoly: #create from A, square
			raise Exception('got too little information about the area, need areaPoly polygon. ')
		self.areaPoly=G.areaPoly
		self.area=polygon_area(G.areaPoly)
		lim=fun.polygonLim(self.areaPoly)
		self.xlim=list(lim[0:2])
		self.ylim=list(lim[2:4])
		if self.G and self.G.gridL:
			self._fixGrid(L=self.G.gridL)
		else:
			self._fixGrid()
		if not generate:
			self._fileini()
		else:
			self.generateTrees(dens, dbh_mean, dbh_std)
	def _fileini(self):
		self.stumpFiles=[151,152,153,154,251,252,451,452,551,552,553,554]
		self.treeFiles=[101,102,103,104,105,106,107,108,109,110,\
						201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,224,225,226,227,\
						302,303,304,305,306,307,308,309,310,311,\
						401,402,403,404,405,406,407,408,409,410,\
						501,502,503,504,505,\
						601,602,603,604]
		self.exceptionalThinning=[ 210]#,, 221]#, 211] #files that have proven to be really good.. unfortunately 211 and 221 are corrupt..
		self.maybeThinning=[102, 103, 403, 105, 304]
		self.thinningFiles=self.exceptionalThinning+self.maybeThinning
	def generateTrees(self,dens, dbh_mean, dbh_std):
		"""
		Generates trees. Can only handle square areas right now.
		This is only done once, so code is not optimized.
		"""
		self.dbh_mean=dbh_mean
		self.dens=dens
		self.dbh_std=dbh_std
		trees=dens*self.area
		idealCell=[5.0,5.0]
		W=self.xlim[1]-self.xlim[0] 
		L=self.ylim[1]-self.ylim[0]
		if self.G:
			self.G.xlim=self.xlim	#override. The polynomial for A given to terrain is master
			self.G.ylim=self.ylim
		cellW=W/float(ceil(W/idealCell[0])) #ideal 5*5m, but adapt to the polygon
		cellL=L/float(ceil(L/idealCell[1]))
		cellA=cellW*cellL
		treesPerCell=int(floor(trees*cellA/self.area)) #if the whole cell is in areaPoly
		xAr=np.arange(self.xlim[0], stop=self.xlim[1], step=cellW) #will omitt last row in x and y, correct
		yAr=np.arange(self.ylim[0], stop=self.ylim[1], step=cellW)
		n=0
		for x in xAr:
			for y in yAr:
				if not col.pointInPolygon((x,y), self.areaPoly): continue
				tint=[]
				neighGrids=self.getNeighborObst([x,y])
				for i in range(treesPerCell):
					r=-1
					while r<0: r=random.normalvariate(dbh_mean, dbh_std)*0.5
					placed=False
					while not placed:
						pos=[x+random.uniform(0,cellW), y+random.uniform(0,cellL)]
						placed=True
						for neigh in tint:
							if getDistance(pos, neigh.pos) < r+neigh.radius:
								placed=False
								break #just for negih, while is still on
						if pos[0]-x<r or pos[0]-x>cellW-r or pos[1]-y<r or pos[1]-y>cellL-r: #iterate through neighboring trees.
							for neigh in neighGrids:
								if getDistance(neigh.pos, pos)<r+neigh.radius: #collide
									placed=False
									break
					t=Tree(pos, r, terrain=self)
					if random.uniform(0,1)<0.4: t.marked=True
					tint.append(t)
					n=n+1
		print "trees left over during generation: ",trees-n, "this is a problem, solve it..."
		"""thoughts around this problem. These trees needs to be distributed inside the cells,
		i.e some cells should have more trees than treesPerCell. In order to have this information,
		the polygon needs to be compared to the "grid area"."""
	def _fixGrid(self,L=5):
		"""
		so, the grid is orginized into a dictionary with tuple (1,1), (1,2) keys and so on format
		with a square uniform grid with side L. Negative numbers are allowed. (0,i),(i,0) are not allowed as a keys
		"""
		self._gridL=int(L)
		#fix the list for neighbors.
		itList=[]
		for dx in [-1,0,1]:
			for dy in [-1,0,1]:
				itList.append((dx,dy))
		self._gridkIter={1:itList} #this dictionary will be used numerous of times, saves calculations
		self._gridLinv=1.0/float(L) #saves time later, will be used alot
		xlim=[int(floor(self.xlim[0]*self._gridLinv)), int(ceil(self.xlim[1]*self._gridLinv))]
		ylim=[int(floor(self.ylim[0]*self._gridLinv)), int(ceil(self.ylim[1]*self._gridLinv))]
		for lst in [xlim, ylim]:
			if 0 in lst:
				i=lst.index(0)
				lst[i]=1 #0 does not exist as an index, always replaced by 1
		self._grid={}
		self._gridIndXlim=xlim
		self._gridIndYlim=ylim
		for x in np.arange(xlim[0], xlim[1]+1): #+1 to not omit the last.
			for y in np.arange(ylim[0], ylim[1]+1):
				if x==0 or y==0: continue
				self._grid[(x,y)]=[]
	def _insertToGrid(self,obst):
		"""inserts into grid"""
		ind=self._getGridIndex(obst.pos)
		obst._gridIndex=ind
		try: self._grid[ind].append(obst)
		except KeyError:
			self._grid[ind]=[obst]
	def _clearGrid(self):
		"""
		removes everything in the grid list
		"""
		for key in self._grid.keys():
			self._grid[key]=[] #cleared
	def _getGridIndex(self,p):
		"""returns grid index of pos p"""
		if p[0]>0: ind1=int(ceil(p[0]*self._gridLinv))
		elif p[0]==0: ind1=1
		else: ind1=int(floor(p[0]*self._gridLinv))
		if p[1]>0: ind2=int(ceil(p[1]*self._gridLinv))
		elif p[1]==0: ind2=1
		else: ind2=int(floor(p[1]*self._gridLinv))
		return (ind1, ind2)
	def remove(self,obst):
		"""
		removes from grid and internal lists.
		This method should always be used instead of e.g. t.obstacles.remove(obst)
		"""
		if obst in self.obstacles: self.obstacles.remove(obst)
		ind=obst._gridIndex
		if not ind:	ind=self._getgridIndex(obst.pos)
		if obst in self._grid[ind]: self._grid[ind].remove(obst)
		list=None
		if isinstance(obst, Tree): list=self.trees
		elif isinstance(obst, Stump): list=self.stumps
		elif isinstance(obst, Root): list=self.roots
		elif isinstance(obst, Hole): list=self.holes
		else: raise Exception('type'+obst.__class__.__name__+' is not supported for removal')
		if obst in list: list.remove(obst)
	def getNeighborObst(self,pos, index=False, Lmax=0):
		"""returns obstacle from the neighboring cells given by Lmax"""
		if index: ind=pos
		else: ind=self._getGridIndex(pos)
		if Lmax>self._gridL: #more than 8 neighbors are needed
			k=int(ceil(Lmax*self._gridLinv+0.5)) #+0.5 since we start from middle of cell
			try:
				itList=self._gridkIter[k]
			except KeyError: #not in dictionary, calculate it brute force
				itList=[(0,0)]
				#this is only done once for each k.
				low=1
				for dx in xrange(0, k+1):
					for dy in xrange(low, k+1):
						if sqrt((abs(dx)-1)**2+(abs(dy)-1)**2)<k: #take adv. of geometry. Only~1/4sqrt.comp.
							#-1 since center of grid is considered (in two grids, 2*-0.5=-1)
							#we want to be on the safe side. dx==0 to avoid (0-1)**2+(0-1)**2
							if dx==0:
								ls=(dx,dy), (dx,-dy) #only two copies here, not four
							elif dy==0:
								ls=(dx,dy), (-dx,dy)								
							else: ls=(dx,dy),(dx,-dy),(-dx,-dy),(-dx,dy)
							for c in ls:
								itList.append(c) 
					low=0 #faster than if before "for dy" 
				#itList finished, add to dictionary for next time
				self._gridkIter[k]=itList
		else: #iterate through 8 neighbors and self
			itList=self._gridkIter[1]
		obstlist=[]
		for dx, dy  in itList:
			indT=(ind[0]+dx, ind[1]+dy) #I am invinsible!!
			try: obstlist.extend(self._grid[indT])
			except KeyError: #thrown if we are outside borders
				pass
		return obstlist
	def old_getNeighborObst(self,pos, index=False, Lmax=0):
		"""should be removed!!!"""
		if index: ind=pos
		else: ind=self._getGridIndex(pos)
		if Lmax>self._gridL: #more than 8 neighbors are needed
			k=int(ceil(Lmax*self._gridLinv+0.5))
			dxL=range(-k, k+1)#the +1 is not included
			dyL=dxL
		else: #iterate through 8 neighbors and self
			dxL=[-1,0,1]
			dyL=[-1,0,1]
		obstlist=[]
		for dx in dxL:
			for dy in dyL:
				#if sqrt(dx**2+dy**2)>k:
					#continue
				indT=(ind[0]+dx, ind[1]+dy) #I am invinsible!!
				try: obstlist.extend(self._grid[indT])
				except KeyError: #thrown if we are outside borders.
					pass
		return obstlist
	def restart(self):
		"""
		resets terrain. E.g. reloads stochastic stumps, removes holes etc.
		"""
		type=[]
		if self.stumps>0:
			type.append('Stump')
		if self.trees>0:
			type.append('Tree')
		self._clearGrid() #clears the grid thing.
		self.obstacles=[]
		self.trees=[]
		self.roots=[]
		self.stumps=[]
		self.holes=[]
		if len(self.obstacles)>0:
			print len(self.obstacles), self.obstacles[0].name, self.obstacles[1].name
			raise Exception('obstacles should be removed by now.')
		if 'Stump' in type:
			self.readStumps()
		if 'tree' in type:
			if self.generate:
				self.generateTrees(self.dens, self.dbh_mean, self.dbh_std)
			else:
				self.readTrees()
	def readStumps(self,B=None, path=None):
		if path:
			B=path
		else:
			if not self.stumpFile: return False
			if len(self.stumps)>0: #remove the old ones
				for s in copy.deepcopy(self.stumps):
					self.remove(s)
				self.stumps=[]
			start=os.path.dirname(os.path.abspath(__file__)) #current folder
			start+='/terrainFiles/planting/SAV-'
			if not B and self.stumpFile == 'undefined':
				self.stumpFile=random.choice(self.stumpFiles)
				B=start+str(self.stumpFile)+'.DAT'
			elif not B:
				B=start+str(self.stumpFile)+'.DAT'
		print "file: %s"%B
 		f = open(B, 'r')
		lines=f.readlines()
		lines=lines[1:]		#omitts the first line
		print "stumps: %d"%len(lines)
		#conversion from matlab script: list indices begin with 
		#0 in python and 1 in matlab, furthermore e.g 9:10 only means 9
		#thus 9:11<=>8:11 etc.
		for line in lines:
			#procedure to skip strings beginning with '0', which python2 reads as octals
			xStr=line[8:11] #xposition in meters
			yStr=line[11:14] #ypos in meters
			dbhStr=line[14:17]#
			specie=line[7]
			while xStr[0] == '0' and len(xStr)>1: xStr=xStr[1:]#skip '0'
			while yStr[0] == '0' and len(yStr)>1: yStr=yStr[1:]
			while dbhStr[0] == '0' and len(dbhStr)>1: dbhStr=dbhStr[1:]
			pos=[eval(xStr)/10.,eval(yStr)/10.] #x and y in m
			dbh=eval(dbhStr)/1000. #dbh in m
			specie=eval(specie)
			if specie==1:
				specie='pine'
			elif specie==2:
				specie='spruce'
			elif specie==3:
				specie='leaf'
			else:
				raise(Exception('readtrees: specie-code %d is not recognized. '%specie))
			s=Stump(pos, dbh=dbh, terrain=self, specie=specie)
		f.close()
	def readTrees(self, path=None, thinning=True):
		"""reads in trees from the specified files."""
		start='terrain/terrainFiles/thinning/GA-'
		uniform=random.uniform
		self.xlim=[0,25] #limits for the files that this method reads.
		self.ylim=[0,40]
		#self.treeFile=215
		if len(self.trees)>0:
			for tree in self.trees:
				self.remove(tree)
			self.trees=[]#needed, stored otherwise in loop
		if not path and self.treeFile == 'undefined':
			if thinning:
				self.treeFile=str(random.choice(self.thinningFiles))
			else:
				self.treeFile=str(random.choice(self.treeFiles))
			path=start+str(self.treeFile)+'.DAT'
		elif not path:
			path=start+str(self.treeFile)+'.DAT'
		#if stand==2:
			#path=start+str(self.treeFile)+'.DAT'
		print "file: %s"%path
		f = open(path, 'r')
		lines=f.readlines()
		print "trees: %d"%len(lines)
		biggestDBH=0
		for line in lines:
			a={} #to be able to modify the arguments... I think... seems like a strange procedure...
			a['xStr']=line[25:28] #xposition in dm
			a['yStr']=line[28:31] #ypos in dm
			a['dbhStr']=line[31:34]#dbh in mm
			a['hStr']=line[39:42]# height in dm
			a['wStr']=line[63:66]# weight in kg
			a['volStr']=line[74:77]#volume with bark, dm3
			a['specieStr']=line[24:25]
			a['logWeight']=line[66:69]
			for s in ['xStr', 'yStr', 'dbhStr', 'hStr', 'wStr', 'volStr', 'logWeight']: #procedure to skip strings beginning with '0', which python2 reads as octals
				if len(a[s])==0: print "Error:",s,a[s], line
				else:
					while a[s][0] == '0' and len(a[s])>1: a[s]=a[s][1:]#skip beginning '0'
			pos=[eval(a['xStr'])/10.,eval(a['yStr'])/10.] #x and y in m
			dbh=eval(a['dbhStr'])/1000. #dbh in m
			height=eval(a['hStr'])/10.	#m
			weight=eval(a['wStr'])	#kg
			vol=eval(a['volStr'])/1000. #m3
			specie=eval(a['specieStr'])
			logWeight=eval(a['logWeight'])
			if specie==1:
				specie='pine'
			elif specie==2:
				specie='spruce'
			elif specie==3:
				specie='leaf'
			else:
				raise(Exception('readtrees: specie-code %d is not recognized. '%specie))
			t=Tree(pos, dbh=dbh,terrain=self, height=height, weight=weight, logWeight=logWeight, vol=vol, specie=specie)
			if dbh>biggestDBH: biggestDBH=dbh
		print "biggest dbh:", biggestDBH, "m"
		self.removeCollisions()
		f.close()
	def removeCollisions(self):
		"""
		Obviously, the guys in the 70's did some mistakes and put their trees at the same positions.
		This method takes care of this by placing the trees really close to each other.
		One can discuss if they should be removed instead..
		"""
		rTemp=0
		thTemp=0
		dth=pi/25.
		b=0.05 #constant for the spiral
		for t in self.trees:
			placed=True
			while placed:
				placed=False
				for t2 in self.trees:
					if t is not t2:
						if col.collide(t,t2, t.pos):
							tpos=t.pos
							while col.collide(t,t2, tpos) or tpos[0]<self.xlim[0] or tpos[0]>self.xlim[1] or tpos[1]<self.ylim[0] or tpos[1]>self.ylim[1]: #trees collide... bad..
								#search in a spiral.
								thTemp+=dth
								rTemp=b*thTemp
								tpos=getCartesian([rTemp, thTemp], origin=t.pos)
							t.pos=tpos
							placed=True #redo again for all trees.				
	def GetVisibleObstacles(self,pos, R):
		#Get obstacles in a radius R from point pos. Optimize: let the obstacles be in a grid and only search those in adjacent grids.
		obstList=self.getNeighborObst(pos, Lmax=R)
		return [o for o in obstList if o.visible and getDistance(pos, o.pos)< o.radius+R]
	def GetTrees(self, pos, R, onlyNonHarvested=False):
		obstList=self.getNeighborObst(pos, Lmax=R)
		if onlyNonHarvested: obstList=[t for t in obstList if isinstance(t,Tree) and not t.harvested]
		return [t for t in obstList if isinstance(t,Tree) and getDistance(pos, t.pos)< t.radius+R]
	def GetRoots(self, pos, R):
		obstList=self.getNeighborObst(pos, Lmax=R)
		return [r for r in obstList if isinstance(r,Root) and getDistance(pos, r.pos)< r.radius+R ]
	def GetStumps(self,pos,R):
		obstList=self.getNeighborObst(pos, Lmax=R)
		return [s for s in obstList if isinstance(s,Stump) and getDistance(pos, s.pos)< s.radius+R ]
	def GetBoulders(self, pos, R, distr='pareto'):
		"""
		we choose to base all this on the volume
		possible distributions: 'exp' or 'pareto'
		"""
		#new, based on volume
		boul=[]
		area=R**2*pi
		uniform=random.uniform
		nboul=int(self.boulderFreq*area)
		thresh=0.125/1000.0 #minimum volume
		meanR=pow(self.meanBoulderV/pi*3.0/4.0, 1.0/3.0) #from mean volume
		meanV=self.meanBoulderV
		Vols=[]
		#randomly dist.
		if distr=='exp':
			lambd=1./(meanV-thresh) #for exponential
			for i in range(nboul):
				vol=thresh+random.expovariate(lambd)
				Vols.append(vol)
		elif distr=='pareto':
			alpha=1.1 #for pareto
			xm=(alpha-1)/alpha*(meanV-thresh) #pareto
			for i in range(nboul):
				vol=thresh+pareto.rvs(alpha, scale=xm)
				Vols.append(vol)
		else:
			raise Exception('unknown distribution %s'%str(distr))
		Vols=sorted(Vols, key=lambda vol: -vol) #sort, biggest first to get it in there..
		for vol in Vols:
			placed = False
			radius=pow(vol/pi*3.0/4.0, 1.0/3.0)
			#end of the new stuff
			#print "A: %f n:%f %f %f %f"%(area, nboul, i, nboul, radius)
			i=0
			while not placed:
				placed=True
				z=-uniform(0+radius,0.2+radius)
				x=uniform(pos[0]-R-radius/2., pos[0]+R+radius/2.)
				y=uniform(pos[1]-R-radius/2., pos[1]+R+radius/2.)
				for o in boul:
					if sqrt(pow(x-o.pos[0],2)+pow(y-o.pos[1],2)+pow(z-o.z,2))<(radius+o.radius):
						placed=False
						#print "i=%f nboul=%f fcould not place boulder b.x:%f b.y:%f b.z: %f x:%f y:%f z:%f"%(i,nboul,o.pos[0],o.pos[1], o.z,x,y,z)
						break
				if placed:
					boul.append(Boulder([x,y],radius,z))
				i+=1
				if i>50: 
					print "could not place all stones.. frequency/area is obviously too big."
					break
		return boul
	def draw(self, ax=None): 
		"""draw to an axis"""
		if ax==None:
			plt.ion()
			fig = plt.figure()
			ax = fig.add_subplot(111,aspect='equal', axisbg='#A2CD5A')
			ax.axis(self.xlim+self.ylim)
			ax.set_xlabel('x (m)')
			ax.set_ylabel('y (m)')	
		for index, o in enumerate(self.obstacles):
			if o.visible: o.draw(ax)
			if index>100000:
				print "too many objects, breaks"
				plt.clf()
				break #to many objects
		#for e in self._grid.keys():
			#ax.plot((e[0]-0.5)*self._gridL, (e[1]-0.5)*self._gridL, 'bo') #middle of cell
	def removeObstacle(self,o):
		"""
		removes the obstacle from all the terrain lists.
		"""
		self.obstacles.remove(o)
		if isinstance(o, Tree):
			self.trees.remove(o)
		elif isinstance(o, Stump):
			self.stumps.remove(o)
		elif isinstance(o, Root):
			self.roots.remove(o)
if __name__=='__main__':
	"""this is a comparison between two getNeighbor functions."""
	random.seed(1)
	tic=time.clock()
	plot=True
	L=100
	lookups=1000
	from sim.tools import globalVar
	G=globalVar()
	G.gridL=2
	t=Terrain(G=G, generate=True, areaPoly=[(0,0), (L,0), (L,L), (0,L)], A=150, dens=2, dbh_mean=0.5, dbh_std=0.1)
	print "time to generate terrain:", time.clock()-tic
	oldtrees=0
	newtrees=0
	random.seed(1)
	tic=time.clock()

	for i in xrange(lookups):
		pos=random.uniform(0,L), random.uniform(0,L)
		Lmax=random.uniform(5,25)
		a=t.old_getNeighborObst(pos, Lmax=Lmax)
		for tree in a:
			oldtrees+=1
			p=1+3*4/2. #just something to represent time consumption for every tree.
	old=time.clock()-tic
	random.seed(1)
	tic=time.clock()
	for i in xrange(lookups):
		pos=random.uniform(0,L), random.uniform(0,L)
		Lmax=random.uniform(5,25)
		a=t.getNeighborObst(pos, Lmax=Lmax)
		for tree in a:
			newtrees+=1
			p=1+3*4/2. #just something to represent time consumption for every tree.
	new=time.clock()-tic
	print "time for %d lookups: old: %f new:%f"%(lookups, old, new)
	print "trees with old method: %e, trees with new method: %e"%(oldtrees, newtrees)
	point=(random.uniform(40,60), random.uniform(40,60))
	Lmax=20
	l1=t.getNeighborObst(point,Lmax=Lmax)
	l2=t.old_getNeighborObst(point,Lmax=Lmax)
	if plot:
		f=plt.figure()
		for l, a in (l1,'121'), (l2, '122'):
			ax=f.add_subplot(a, aspect='equal', axisbg='#A2CD5A')
			c=Circle(point, 20, facecolor='b', alpha=300)
			ax.add_patch(c)
			for tree in l:
				tree.color='r'
			t.draw(ax)
			plt.plot(point[0], point[1], 'co')
			ax.set_ylim(point[1]-35, point[1]+35)
			ax.set_xlim(point[0]-35, point[0]+35)
			for tree in l:
				tree.color='g'
		plt.show()
