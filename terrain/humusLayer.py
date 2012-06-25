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
	"""
	def __init__(self, terrainAP=None, rasterDist=1.0, humusType=None):
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
		for i in range(self.xNumInv):
			ztemp=[]
			for j in range(self.yNumInv):
				ztemp.append(random.triangular(dPar[0],dPar[1],dPar[2]))
			z.append(ztemp)
		self.z=np.array(z)
		self.interpolDepth=RectBivariateSpline(x,y,self.z)
		#That was the initiation of the humuslayer

	def getDepth(self,pos):
		xpos=pos[0]
		ypos=pos[1]
		depth=float(self.interpolDepth.ev(xpos,ypos))
		#print 'Depth of humuslayer is:', depth
		return depth

	def plotIt(self):
		self.z=self.z.transpose()
		plt.imshow(self.z, origin='lower',cmap='YlOrBr',extent=[0,50,0,40])
		plt.xlabel('x[m]')
		plt.ylabel('y[m]')
		plt.title('Depth of humus layer [m]')
		plt.colorbar()

		plt.figure(2)
		self.z=self.z.transpose()
		xway=range(0,51,1)
		xway=np.linspace(0,50,100)
		z=[]
		for i in range(len(xway)):
			ztemp=[]
			yway=np.linspace(0,40,8)
			for j in range(len(yway)):
				print i,j
				ztemp.append(float(self.getDepth([xway[i],yway[j]])))
			z.append(ztemp)
		self.z=np.array(z)
		self.z=self.z.transpose()
		plt.imshow(self.z, origin='lower',cmap='YlOrBr',extent=[0,50,0,40])
		plt.xlabel('x[m]')
		plt.ylabel('y[m]')
		plt.title('Depth of humus layer [m] with interpol')
		plt.colorbar()

		plt.show()
	
	def draw(self):
		pass
		
if __name__=='__main__':
	terrainAP=[(0,0),(50,0),(50,40),(0,40)]
	hL=HumusLayer(terrainAP=terrainAP,rasterDist=1, humusType='3')
	hL.plotIt()
