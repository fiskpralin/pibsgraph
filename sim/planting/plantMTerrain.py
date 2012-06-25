
#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
import random

#from terrain.obstacle import Obstacle
from terrain.terrain import *
from collision import *



class PlantMTerrain(Terrain):
	"""
	This class is for the planting machine terrain.

	Changes all the time due to new instructions, hard to keep it backwards compatible.

	It is somewhat important in what order different constituents are generated. Should boulders,
	stumps(with roots) or surfaceboulders be generated first?

	surfaceboulder=True should be removed from the code below...

	"""
	def __init__(self, G=None, ttype=None, areaPoly=None):
		if not areaPoly:
			areaPoly=[(0,0), (50,0), (50, 40), (0,40)] #default for stump-files.
		Terrain.__init__(self,G, areaPoly=areaPoly) 
		self.stumpFile='undefined'
		self.treeFile='undefined'
		if str(ttype)=='random':
			choices=['0','1','2', '3', '4', '5']
			ttype=random.choice(choices)
		elif ttype is None:
			ttype='3' #default This has now been changed from 5 which it was before article 3
		else:
			ttype=str(ttype)
		self.ttype=ttype
		if ttype=='0':
			self.stumpFile = None #afforestration
			self.stumpsPerH = 0
			self.boulderFreq = 0
			self.meanBoulderV = 0
			self.blockQuota = 0
			self.groundModel = 0  #Anderssons
			self.humusType = '2'
			self.surfaceBoulders= False
			
		elif ttype=='1':
			self.stumpFile = 554
			self.stumpsPerH = 230
			self.boulderFreq = 28
			self.meanBoulderV = 0.9/1000.#dm3-m3
			self.blockQuota = 0.25
			self.groundModel = 4  #Anderssons
			self.humusType = '2'
			self.surfaceBoulders= True
			
		elif ttype=='2':
			self.stumpFile = 554
			self.stumpsPerH = 230
			self.boulderFreq = 28
			self.meanBoulderV = 0.9/1000.#dm3-m3
			self.blockQuota = 0.25
			self.groundModel = 4  #Anderssons
			self.humusType = '4'
			self.surfaceBoulders= True
			
		elif ttype=='3':
			self.stumpFile = 452
			self.stumpsPerH = 730
			self.boulderFreq = 43
			self.meanBoulderV = 1.5/1000.
			self.blockQuota = 0.55
			self.groundModel = 3  #Anderssons
			self.humusType = '3'
			self.surfaceBoulders= True
			
		elif ttype=='4':
			self.stumpFile = 553
			self.stumpsPerH = 635
			self.boulderFreq = 23
			self.meanBoulderV = 4.3/1000.
			self.blockQuota = 0.75
			self.groundModel = 2  #Anderssons
			self.humusType = '2'
			self.surfaceBoulders= True
			
		elif ttype=='5': #medelhygget
			self.stumpFile = 553
			self.stumpsPerH = 635
			self.boulderFreq = 23
			self.meanBoulderV = 4.3/1000.
			self.blockQuota = 0.75
			self.groundModel = 1  #Anderssons
			self.humusType = '4'
			self.surfaceBoulders= True
		else:
			raise Exception("ttype %s not correct"%(str(ttype),))
		if self.stumpFile=='undefined': #default..
			self.stumpFile='554'
		if self.humusType: self.makeHumusLayer()
		if self.stumpFile: self.readStumps()
		if self.surfaceBoulders: self.makeSurfaceBoulders()
		print "Terrain is initialized. Ttype: %s, SurfaceBoulders: %s"%(ttype,str(self.surfaceBoulders))
		
if __name__=="__main__":
	"""example code:"""
	terrain=Terrain()
	terrain.readTrees() #random
	terrain.makeSurfaceBoudlers() #from file
	terrain.draw()
	plt.show()
