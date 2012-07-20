import os, sys #insert /dev to path so we can import these modules.
if __name__=='__main__':
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RectBivariateSpline 
import time
import graph_alg.GIS.GIS as GIS #Gis folder -> GIS.py
from math import *
import graph_alg.draw as draw
from functions import getCartesian as getCa,getCylindrical as getCy
###
#This file is supposed to be a script that introduces how the interpolation and plotting works.
#I have taken out a lot of stuff that is implemented in the ExtendedGraph class, just to show the methods I've used so that everything is in one place..
###


#this is a sweref99 coordinate outside of sandviken. Needed in order to use local coordinates
globalOrigin=596673,6728492

#define our area...local coordinates.. a polygon (actually rectangle here) with 4 points

areaPoly=[(0, 0), (240, 0), (240, 480), (0, 480)]

print areaPoly

#get our x,y,z raster data for given area. numpy arrays
#I simply read some data from the GIS folder. See GIS/grid/.. for the files used.
#readTerrain function is very simple, what you need to know is that it scan's through the .asc x,y,z data and stores all points that are inside areaPoly polygon.
t_x,t_y,t_z=GIS.readTerrain(globalOrigin=globalOrigin, areaPoly=areaPoly)

#draw... some routines I have written for plotting... simple to use
fig=plt.figure()
ax=fig.add_subplot(121)
ax=draw.plotBackground(globalOrigin=globalOrigin , areaPoly=areaPoly, ax=ax)
#above localizes us in the flight photo 672_59_11 found in GIS folder and uses photo as background
ax=draw.plot2DContour(t_x,t_y,t_z,ax, w=2) #plots contours

# A naive and simple method for getting the roll of a road segment.
def naiveroll():
	"""plot ugly sometimes. depending on direction"""
	roadwidth=4.0
	alpha=atan((p2[1]-p1[1])/(p2[0]-p1[0])) #this or
	alpha2=atan2((p2[0]-p1[0]),(p2[1]-p1[1])) #this?
	print 180/pi*alpha, 180/pi*alpha2
	length=sqrt((p2[1]-p1[1])**2+(p2[0]-p1[0])**2)
	p11=getCa([-roadwidth/2,0], direction=pi/2.-alpha2, origin=p1,fromLocalCart=True)
	p12=getCa([roadwidth/2,0], direction=pi/2.-alpha2, origin=p1,fromLocalCart=True)
	p21=getCa([-roadwidth/2,0], direction=pi/2.-alpha2, origin=p2,fromLocalCart=True)
	p22=getCa([roadwidth/2,0], direction=pi/2.-alpha2, origin=p2,fromLocalCart=True)
	x1=np.linspace(p11[0], p21[0], points)
	x2=np.linspace(p12[0], p22[0], points)
	y1=np.linspace(p11[1], p21[1], points)
	y2=np.linspace(p12[1], p22[1], points)
	z1=interpol.ev(x1,y1)
	z2=interpol.ev(x2,y2)
	roll=[]
	for ent in range(len(z1)):
		roll.append(180*(1/pi)*atan((z2[ent]-z1[ent])/roadwidth))
	return roll, p11, p12

# A naive and simple method for getting the roll of a road segment.
def GISroll():
	"""Implements the method commercial GISsoftware uses to evauate the slope """
	roadwidth=4.0
	alpha=atan2((p2[0]-p1[0]),(p2[1]-p1[1]))
	length=sqrt((p2[1]-p1[1])**2+(p2[0]-p1[0])**2)
	p11=getCa([-roadwidth/2,0], direction=pi/2.-alpha, origin=p1,fromLocalCart=True)
	p12=getCa([roadwidth/2,0], direction=pi/2.-alpha, origin=p1,fromLocalCart=True)
	p21=getCa([-roadwidth/2,0], direction=pi/2.-alpha, origin=p2,fromLocalCart=True)
	p22=getCa([roadwidth/2,0], direction=pi/2.-alpha, origin=p2,fromLocalCart=True)
	x1=np.linspace(p11[0], p21[0], points)
	x2=np.linspace(p12[0], p22[0], points)
	y1=np.linspace(p11[1], p21[1], points)
	y2=np.linspace(p12[1], p22[1], points)
	z1=interpol.ev(x1,y1)
	z2=interpol.ev(x2,y2)
	roll=[]
	for ent in range(len(z1)):
		print ent
		if ent==0: continue
		elif ent==len(z1)-1: break
		roll.append(180*(1/pi)*atan(((z2[ent-1]+2*z2[ent]+z2[ent+1])-(z1[ent-1]+2*z1[ent]+z1[ent+1]))/(8*roadwidth/2.)))
	#to get the correct dimensions for plotting
	roll.insert(0,roll[0])
	roll.insert(-1,roll[-1])
	return roll,p11,p12



#interpolate...I use a scipy interpolator called RectBivariateSpline.. google it for more info.
xlist=t_x[:,0] #just the variations needed, not the 2D-matrix
ylist=t_y[0,:]
interpol=RectBivariateSpline(xlist, ylist, t_z,s=100) #used pretty much everytime we need the height of a specific point. Implemented in fortran and very fast. If we want smoothing we can use s=500 or similar


#we now have an interpolation and can use interpol to get height in pretty much every point possible. 

#two arbitrary points:
#p1=(200.0,150.0 )
#p2=(230.0,480.454)
#p1=(200.89,300.5664)
#p2=(100.45,480.78)
p2=(150.5,480.1)
p1=(100.56,50.4)
points=200 #a lot of points along above line.
x=np.linspace(p1[0], p2[0], points)
y=np.linspace(p1[1], p2[1], points)
z=interpol.ev(x,y) #gives array of z for x,y list of positions. we are done with the pitch more or less

#roll,p11,p12 = naiveroll()
roll,p11,p12 = GISroll()
plt.plot(x,y, 'o') #plot the points defined by x, y arrays
plt.plot(p12[0],p12[1],'o')
plt.plot(p11[0],p11[1],'o')
ax2=fig.add_subplot(222)
d=np.sqrt(x**2+y**2)-np.sqrt(x[0]**2+y[0]**2) #distance from start point
ax2.set_xlabel('d')
ax2.plot(d,z,lw=2) #plot z along line...
ax2.set_title('The interpolated height along the line specified to the left.')
ax3=fig.add_subplot(224)
ax3.set_xlabel('d')
ax3.plot(d,roll,lw=2) #plot z along line...
ax3.set_title('The roll with positive as falling left and negative as falling right')

plt.show()

#we are done and have z data for this point. What we want to do is to given t_x,t_y and t_z data above get the gradient or roll for an arbitrary point..


