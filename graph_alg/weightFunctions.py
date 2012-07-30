
import numpy as np
import functions as fun

from math import atan, pi
import operator
from terrain_tools import getSimplePitch, getRoll #doesn't work

#################
#Below is a number of weight functions
##################

def weightFromLength(R,x,y,z):
	p1=x[0], y[0]
	p2=x[-1], y[-1]
	c=fun.getDistance(p1,p2)
	assert c>=0
	return c

def weightFunctionFirst(R,x,y,z):
	"""
	a weight function that has 3 vectors with positions as input and returns a weight
	that is independent of the sampling rate etc.

	The first one is just a simple test.
	"""
	d=fun.getDistance((x[0], y[0]), (x[-1], y[-1]))
	mu= np.mean(z)
	w=d+sum(abs(z-mu))/len(z)
	return w

def normalizedPitchDist(R, x,y,z):
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

def normPitchRollDist(R,x,y,z):
	"""
	Still only uses the maximum value of the pitch and the roll. The weight of each edge
	need be modeled in a way thought through thoroughly.
	"""
	inf=1e9
	A=0.3 #Pitch
	B=0.3 #Roll
	C=1.0-A-B #Distance
	pitchlim=18.4 #deg This limit is approximate
	rollim=11.3 #deg This limit is approximate
	p1 = (x[0],y[0]) 
	p2 = (x[-1],y[-1])
	d = fun.getDistance(p1,p2)
	points = max(10,int(d)) #every meter
	rollist = getRoll(R, p1,p2,points=points,style='weighted')
	pitchlist = getSimplePitch(R,p1,p2,points=points)
	rollmax = max(np.abs(np.array(rollist)))#Doesnt work fix with np
	pitchmax = max(np.abs(np.array(pitchlist)))#Doesnt work fix with np
	if rollmax > rollim or pitchmax > pitchlim: return inf
	w = d*(A*pitchmax/pitchlim+B*rollmax/rollim+C)
	return w

