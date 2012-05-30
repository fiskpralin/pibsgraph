import numpy as np
import os
if __name__=='__main__':
	import sys #insert /dev to path so we can import these modules.
	cmd_folder = os.path.split(os.path.split(os.path.dirname(os.path.abspath(__file__)))[0])[0]
	print cmd_folder
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
from string import split

import collision as col
import functions as fun

"""
This module provides a number of GIS-related routines.
"""

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
	xrange=[xrange[0]-2, xrange[1]+2]
	yrange=[yrange[0]-2, yrange[1]+2]
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
			xtmp=int(l[0])
			ytmp=int(l[1])
			if ytmp>yrange[1]: break #because of how file is organized, we can do this.
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
					xlist, ylist, zlist = [], [], []
				xlist.append(xtmp)
				ylist.append(ytmp)
				zlist.append(ztmp)
				yold=ytmp
		f.close()
	x-=globalOrigin[0]
	y-=globalOrigin[1]
	x=np.transpose(x) #we want it in the standard form that scipy understands.
	y=np.transpose(y) #this form corresponds to numpy.mgrid standard, but not numpy.meshgrid
	z=np.transpose(z)
	return x,y,z


