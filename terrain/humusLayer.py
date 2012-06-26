#!/usr/bin/env python
from math import *
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import random
from scipy.interpolate import RectBivariateSpline

class HumusLayer():
	"""
	Needs comments and descriptions as well as nice plotting feature for determination of rasterDist,
	done but not working...
	This class is the HumusLayer that can be implemented for any type of machine simulation. However it is
	of largetst interest for the plantng machine which interacts with the soil. The humustypes are defined
	from data given by Back Tomas Ersson prior to the work on article 3. We use 2D interpolation with cubical
	splines from scipy.interpolate in order to be able to tell the depth in any given position, done in getDepth.
	"""
	def __init__(self, terrainAP=None, rasterDist=1.0, humusType=None):
		"""
		The humus layer has a triangular thickness distribution with parameters in dPar.
		RasterDist is the distance between each node on our raster with thicknesses given by the distribution.
		A large distance mimics a situation where the humuslayer is more or less spatially invariant,
		whilst a small distance mimics a situation where the thickness varies on a small length scale.
		"""
		self.color='Tan'
		if humusType=='2': dPar=[0.00,0.05,0.01]#triangular distribution [min,max,mode] meters
		elif humusType=='3': dPar=[0.05,0.15,0.10]
		elif humusType=='4': dPar=[0.15,0.30,0.22]
		else: raise Exception('HumusType is not correct.')
		xLim = terrainAP[1][0]
		yLim = terrainAP[3][1]
		self.xNumInv = int(xLim/rasterDist)
		self.yNumInv = int(yLim/rasterDist)
		z=[]
		self.pointDepth = 1
		x=np.linspace(0,xLim,self.xNumInv)
		y=np.linspace(0,yLim,self.yNumInv)
		#These forloops are in order to achieve a two dimensional array with thicknesses
		for i in range(self.xNumInv):
			ztemp=[]
			for j in range(self.yNumInv):
				ztemp.append(random.triangular(dPar[0],dPar[1],dPar[2]))
			z.append(ztemp)
		self.z=np.array(z)#Must be a numpy-array for the interpolation to work.
		self.interpolDepth=RectBivariateSpline(x,y,self.z)
		#That was the initiation of the humuslayer

	def getDepth(self,pos):
		"""
		This method returns the depth at a given position given as self.getDepth([x,y]). Used when each
		surfaceBoulder is placed for example. So it is important that this routine is fast.
		"""
		xpos=pos[0]
		ypos=pos[1]
		depth=float(self.interpolDepth.ev(xpos,ypos))
		#print 'Depth of humuslayer is:', depth
		return depth

	def draw(self):
		"""
		Not much to plot here yet. Possibly we could add plotIt in some sense to get a picture of the humuslayer
		in the background instead of light green flat meadow.
		"""
		self.z=self.z.transpose()
		xway=np.linspace(0,50,100)
		z=[]
		for i in range(len(xway)):
			ztemp=[]
			yway=np.linspace(0,40,80)
			for j in range(len(yway)):
				ztemp.append(float(self.getDepth([xway[i],yway[j]])))
			z.append(ztemp)
		self.z=np.array(z)
		self.z=self.z.transpose()
		self.visibleLayer = plt.imshow(self.z, origin='lower',cmap='YlOrBr',extent=[0,50,0,40])
		plt.xlabel('x[m]')
		plt.ylabel('y[m]')
		#plt.title('Planting domain')
		#plt.colorbar()
		
if __name__=='__main__':
	terrainAP=[(0,0),(50,0),(50,40),(0,40)]
	hL=HumusLayer(terrainAP=terrainAP,rasterDist=1, humusType='3')
	hL.draw()
	plt.show()
