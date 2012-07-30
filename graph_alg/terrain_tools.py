import numpy as np
from math import *

import functions as fun

"""
This file contains a bunch of functions that are used to evaluate the terrain data.
From here we can get the roll, pitch or height along certain lines, in a point or whatever.
"""


def getSimplePitch(R,p1,p2,points=20):
	"""
	Gives the pitch along a line from point p1 to point p2.
	output:
	pitch - a list with the pitch in every point on the line
	"""
	if not R: raise Exception('getSimplePitch need the graph R in order to have the interpolation of terrain data.')
	pitch=[]
	x=np.linspace(p1[0], p2[0], points)
	y=np.linspace(p1[1], p2[1], points)
	z=R.interpol.ev(x,y)
	length=fun.getDistance(p1,p2)
	piInv=1/pi
	for ent in range(len(z)):
		if ent==0:
			pitch.append(180*piInv*atan2(z[ent+1]-z[ent],length/float(points))) #forward difference
		elif ent==len(z)-1:
			pitch.append(180*piInv*atan2(z[ent]-z[ent-1],length/float(points))) #backward difference
			break
		else:
			pitch.append(180*piInv*atan2(z[ent+1]-z[ent-1],2*length/float(points))) #central difference
	if len(pitch)!=len(z): raise Exception('Something wrong with the dimensions of the pitch')
	return pitch

def getRoll(R,p1,p2,points=20,style='naive'):
	"""
	Provided choice of style is 'naive' or 'weighted', this method outputs the roll along a
	road specified by the start point p1 and end point p2.
	"""
	if not R: raise Exception('getRoll need the graph R to get interpolation of the terrain data.')
	alpha=atan2((p2[0]-p1[0]),(p2[1]-p1[1]))
	length=fun.getDistance(p1,p2)
	piInv=1/pi
	roll=[]
	p11=fun.getCartesian([-R.roadWidth*0.5,0], direction=pi*0.5-alpha, origin=p1,fromLocalCart=True)
	p12=fun.getCartesian([R.roadWidth*0.5,0], direction=pi*0.5-alpha, origin=p1,fromLocalCart=True)
	p21=fun.getCartesian([-R.roadWidth*0.5,0], direction=pi*0.5-alpha, origin=p2,fromLocalCart=True)
	p22=fun.getCartesian([R.roadWidth*0.5,0], direction=pi*0.5-alpha, origin=p2,fromLocalCart=True)
	x1=np.linspace(p11[0], p21[0], points)
	x2=np.linspace(p12[0], p22[0], points)
	y1=np.linspace(p11[1], p21[1], points)
	y2=np.linspace(p12[1], p22[1], points)
	z1=R.interpol.ev(x1,y1)
	z2=R.interpol.ev(x2,y2)
	if style=='naive':#This is the most naive method, called naiveroll in intro_interpol.py
		for ent in range(len(z1)):
			roll.append(180*piInv*atan2((z2[ent]-z1[ent]),R.roadWidth))
	elif style=='weighted':#This method was originally called GISroll or GIScopy-method in intro_interpol.py
		for ent in range(len(z1)):
			if ent==0:
				roll.append(180*piInv*atan2(((2*z2[ent]+z2[ent+1])-(2*z1[ent]+z1[ent+1])),6*R.roadWidth*0.5))
			elif ent==len(z1)-1:
				roll.append(180*piInv*atan2(((z2[ent-1]+2*z2[ent])-(z1[ent-1]+2*z1[ent])),6*R.roadWidth*0.5))
				break
			else:
				roll.append(180*piInv*atan2(((z2[ent-1]+2*z2[ent]+z2[ent+1])-(z1[ent-1]+2*z1[ent]+z1[ent+1])),8*R.roadWidth*0.5))
	else: raise Exception('getRoll need a correct style to be supplied at call')
	return roll
	
def getLineElevationCurve(R,p1,p2, points=10):
	"""
	interpolates from the terrain data and returns line coordinates.
	p1-start point
	p2-end point
	output:
	x - array of x-values along line
	y - array of y-values along line
	"""
	x=np.linspace(p1[0], p2[0], points)
	y=np.linspace(p1[1], p2[1], points)
	return x, y, R.interpol.ev(x,y) #evaluate interpolation at above points.

def getPointAltitude(R,p):
	"""
	interpolates from the terrain data and returns point coordinates.
	not tested yet
	"""
	x=np.array(p[0])
	y=np.array(p[1])
	return list(R.interpol.ev(x,y)) #evaluate interpolation at above points.
