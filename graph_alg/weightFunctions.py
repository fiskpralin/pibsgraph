import numpy as np
import functions as fun

from math import atan, pi
import operator

#################
#Below is a number of weight functions
##################
def weightFunctionFirst(x,y,z):
	"""
	a weight function that has 3 vectors with positions as input and returns a weight
	that is independent of the sampling rate etc.

	The first one is just a simple test.
	"""
	d=fun.getDistance((x[0], y[0]), (x[-1], y[-1]))
	mu= np.mean(z)
	w=d+sum(abs(z-mu))/len(z)
	return w

def normalizedPitchDist(x,y,z):
	"""
	see the documentation.
	Look at the two extreme values for the pitch

	roll not in there yet. Also does only care for the extreme values.. not really optimal

	"""
	A=0.3 #importance of distance compared to pitch
 	B=1.0-A
	alim=15*pi/180.0
	imin, zmin = min(enumerate(z), key=operator.itemgetter(1))
	imax, zmax = max(enumerate(z), key=operator.itemgetter(1))
	
	alpha=atan(zmax-zmin)/fun.getDistance((x[imin], y[imin]), (x[imax], y[imax]))
	if alpha>alim: return 1e15
	d=fun.getDistance((x[0], y[0]), (x[-1], y[-1]))
	w=d*(A+B*alpha/alim)
	return w 
