#!/usr/bin/env python
from math import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path
import numpy as np
import random
from scipy.interpolate import RectBivariateSpline

class HumusLayer():
	def __init__(self, terrain=None, rasterDist=1.0, humusType=None):
		print 'This the start of the HumusLayer'
		self.color='Tan'
		if humusType=='2': dPar=[0.00,0.05,0.01]#triangular distribution [min,max,mode] meters
		elif humusType=='3': dPar=[0.05,0.15,0.10]
		elif humusType=='4': dPar=[0.15,0.30,0.22]
		else: raise Exception('HumusType is not correct.')
		xLim = terrain.areaPoly[1][0]
		yLim = terrain.areaPoly[3][1]
		xNumInv = int(xLim/rasterDist)
		yNumInv = int(yLim/rasterDist)
		z=[]
		self.pointDepth = 1
		x=np.linspace(0,xLim,xNumInv)
		y=np.linspace(0,yLim,yNumInv)
		#print x.size, y.size
		for i in range(xNumInv):
			ztemp=[]
			for j in range(yNumInv):
				ztemp.append(random.triangular(dPar[0],dPar[1],dPar[2]))
			z.append(ztemp)
		z=np.array(z)
		#print z.shape
		self.interpolDepth=RectBivariateSpline(x,y,z)
		#That was the initiation of the humuslayer

	def getDepth(self,pos):
		xpos=pos[0]
		ypos=pos[1]
		depth=self.interpolDepth.ev(xpos,ypos)
		print 'Depth of humuslayer is:', depth
		return depth
		
	def draw(self):
		pass
		
