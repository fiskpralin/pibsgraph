from mpl_toolkits.mplot3d import axes3d, Axes3D # <-- NOTE!
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.image as mpimg
from matplotlib.patches import Polygon
import os
if __name__=='__main__':
	import sys #insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0])[0]
	print cmd_folder
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
from matplotlib import cm
from string import split

import collision as col
import functions as fun

#open

def getGlobalCoordinate(globalOrigin, localPos=None, areaPoly=None):
	"""
	returns the global 2D coordinates, that is sweref99.
	"""
	if localPos and areaPoly: raise Exception('cannot give both pos and areaPoly to function')
	if localPos:
		return (globalOrigin[0]+localPos[0],globalOrigin[1]+localPos[1]) #same format, meters.
	else: #transform areaPoly
		pol=[]
		for i in range(len(areaPoly)):
			pol.append((globalOrigin[0]+areaPoly[i][0],globalOrigin[1]+areaPoly[i][1] )) #same format, meters.
		return pol
def getFileList(areaPoly):
	"""
	returns a list of the files that contains the data for the positions.

	fileformat varies between .tab, .asc and .tif, as does the folders but the names are the same.
	
	areaPoly should be given in sweref99 2D coordinates.
	"""
	fileList=['67250_5950_25','67250_5975_25', '67275_5950_25', '67275_5975_25']
	cornerList=[(595000, 6725000),(597500, 6725000), (595000,6727500),(597500,6727500) ]
	w=2500 #m width of cell file is representing
	h=2500

	pol=fun.Polygon(areaPoly[0], areaPoly)
	files=[] #the files we should return.
	for i in range(len(cornerList)):
		pos=cornerList[i]
		nodes=[pos]
		square=[pos, (pos[0]+w, pos[1]), (pos[0]+w, pos[1]+w), (pos[0], pos[1]+w)]
		sq=fun.Polygon((pos[0]+w*0.5, pos[1]+h*0.5), square)
		if col.collide(pol, sq):
			files.append(fileList[i])

	if len(files)==0: raise Exception('could not find matching terrain files')
	return files

	
def readTerrain(globalOrigin=None, areaPoly=None):
	"""
	reads in data and returns local grid  x,y,z coordinates.

	Notes:
	-sweref koordinates are in m, useful. We just need to normalize them to our new origin.
	-we need to work exclusively with numpy arrays, this is heavy stuff.
	-Problem with intersections between the files... how to handle?
	"""
	if not globalOrigin: globalOrigin=(596120, 6727530) #located on map.. nice position..
	if not areaPoly: areaPoly=[(0,0), (200,0), (200,200), (0,200)] #default..
	globalPoly=getGlobalCoordinate(globalOrigin, areaPoly=areaPoly)
	corn=(595000, 6725000)
	bordersPoly=[corn, (corn[0]+5000, corn[1]), (corn[0]+5000, corn[1]+5000), (corn[0], corn[1]+5000)]
	#check if polygon is inside polygon
	for node in globalPoly:
		if not col.pointInPolygon(node, globalPoly): raise Exception('areaPolygon is outside of the area that we have maps for.')

	#so, read in the data for our areaPolygon, or rather square containing it, and
	#normalize the cooordinates with respect to our origin.
	range=fun.polygonLim(globalPoly)
	xrange=range[0:2]
	yrange=range[2:4]
	#now, we should find out which files we need...
	fileList=getFileList(globalPoly)
	x, y, z=None, None, None #will later be created
	xlist, ylist, zlist=[], [], []
	m=1 #modulu, to reduce the grid resolution.. should be set from area of polygon..
	yold=None
	if len(fileList)>1: raise Exception('we do not support file overlaps right now..')
	folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tab')
	for fname in fileList:
		f=open(os.path.join(folder, fname+'.asc'))
		for index, line in enumerate(f):
			l=line.split()
			if len(l)==0: break
			xtmp=int(l[0])
			ytmp=int(l[1])
			if ytmp>yrange[1]: break
			if xtmp<=xrange[1] and xtmp>=xrange[0] and ytmp>=yrange[0] and xtmp%m==0 and ytmp%m==0:
				if yold==None: yold=ytmp
				ztmp=float(l[2])
				if ytmp != yold: #new row..
					#identify if we're gonna need the next file in line..
					if x==None:
						x=np.array(xlist)
						y=np.array(ylist)
						z=np.array(zlist)
					else:
						x=np.vstack((x, xlist))
						y=np.vstack((y, ylist))
						z=np.vstack((z, zlist))
					#y.append(ylist)
					#z.append(zlist)
					xlist, ylist, zlist = [], [], []
				xlist.append(xtmp)
				ylist.append(ytmp)
				zlist.append(ztmp)
				yold=ytmp
		f.close()
	x-=globalOrigin[0]
	y-=globalOrigin[1]
	return x,y,z
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
def plot2DContour(x=None,y=None,z=None, ax=None):
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
	c=ax.contour(x,y,z, zdir='z', linewidths=1, levels=levels,cmap=cm.autumn)
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
	bg=mpimg.imread(os.path.join(folder,'672_59_11.tif'))
	im = plt.imshow(bg, cmap='gray',origin='lower', extent=limits)
	
	#now, adjust to our polygon..
	limits=list(fun.polygonLim(areaPoly))
	w=max(10, max(limits)/10.)
	limits[0]-=w #to get some space to polygon
	limits[1]+=w
	limits[2]-=w
	limits[3]+=w
	ax.axis(limits)
	return ax

	
if __name__=='__main__':
	import cProfile
	globalOrigin=(596120, 6727530) #located on map.. nice position..
	areaPoly=[(0,0), (1000,100), (1000,600), (700,1000), (300,1000)]
	#cProfile.run("readTerrain(globalOrigin=globalOrigin, areaPoly=areaPoly)")
	x,y,z=readTerrain(globalOrigin=(596120, 6727530) , areaPoly=areaPoly)
	#ax=plotSurface(*coord)
	ax=plotBackground(areaPoly, globalOrigin=globalOrigin)
	plot2DContour(x,y,z,ax)
	pol=Polygon(areaPoly, closed=True, color='none', ec='k',lw=3, ls='solid')
	ax.add_patch(pol)
	plt.show()
