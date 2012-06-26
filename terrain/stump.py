#!/usr/bin/env python
from math import *
from matplotlib.patches import Circle
from collision import *
from functions import *
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.path import Path

from obstacle import Obstacle
from root import Root
print type(Obstacle)
class Stump(Obstacle):
	def __init__(self, pos,terrain=None, dbh=0, specie=None):
		Obstacle.__init__(self, pos, isSpherical=True, radius=dbh,terrain=terrain, color='#5C3317')
		self.z=0
		self.dbh=dbh
		self.specie=specie
		self.alpha=1
		self.rLim=0.01 #smaller than 1cm radius is not relevant.
		self.zlim=0.2 #roots deeper than this are discarded.
		if terrain:
			self.name="stump%d"%len(terrain.stumps)
			terrain.stumps.append(self)
		else: self.name="stump"
		#the new stuff! Exciting
		self.RPradius=dbh/2.+0.5   #50^2/10000 =>m2, =(50/100)=0.5^2 dvs 0.5m i grundradie
		self.radius=self.RPradius
		nRoots=int(round(1.822+(0.019*dbh*1000)))
		#let's put up the rooots!
		th=random.uniform(0,2*pi)
		dth=2*pi/float(nRoots)
		cart=getCartesian
		maxdist2=0
		self.roots=[]
		for i in range(nRoots):
			rootR=abs((-0.18943+2.96*dbh)/nRoots/2.) #radius of root
			#taper factor:
			tf=0.04+0.0206*random.normalvariate(0,1)# dm/dm root length.. thus unitless
			if tf<0.0194: tf=0.04
			if self.specie=='pine':
				rootL=abs(random.normalvariate(0.510, sqrt(0.671))) #length of root
			elif self.specie=='spruce':
				rootL=abs(random.normalvariate(0.480, sqrt(0.549))) #length of root
			elif self.specie=='leaf':
				rootL=abs(random.normalvariate(0.510, sqrt(0.658))) #length of root
			else:
				raise Exception('root did not recognize specie:%s'%self.specie)
			#finalR=rootR-rootL*tf
			finalR=rootR*(1-tf)
			alpha=asin(rootR/self.RPradius)#half angle between p1 and p4
			dr=rootR-finalR
			start=cart([self.RPradius-rootR, th],direction=pi/2., origin=self.pos)
			p1=cart([self.RPradius-rootR, th-alpha],direction=pi/2., origin=self.pos)
			comp=rootR/cos(th-pi/2) #to compensate for that the origin of the root is inside the root plate, -rootR above.
			p2=cart([-dr ,rootL+comp],origin=p1, direction=th, fromLocalCart=True)
			p3=cart([-dr-2*finalR,rootL+comp],origin=p1, direction=th, fromLocalCart=True)
			middle=cart([-dr-finalR,rootL],origin=p1, direction=th, fromLocalCart=True)
			p4=cart([-2*rootR,0],origin=p1, direction=th, fromLocalCart=True)
			if dr+finalR-rootR>rootR/100.: raise Exception('Root model does not work as planned. %f, %f, %f'%(rootR, finalR, dr))
			if self.terrain.humusLayer: firstRootVisible=False
			else: firstRootVisible=False
			self._branch(start, rootR, th=th, zin=0, first=firstRootVisible) #makes a branch. May create more sub-branches
			th+=dth
		if self.terrain:
			self.terrain.obstacles.remove(self)
			self.terrain.obstacles.append(self) # to get after the roots in the list.
	def _branch(self,pos,Rin, th, zin, first=False):
		"""takes a position and makes new roots by splitting."""
		cart=getCartesian
		[nSubRoots, newtf, qList, degree, thM, thS]=self._numberOfSubRoots() #q-factor in list, tf, number of subroots return 
		angles=[]
		#degree=random.normalvariate(12,14.8)*2*pi/360. #vertical degree, will later be translated to z-coordinate
		#if degree>0: degree=0.1 #just below surface..
		origin=pos
		for q in qList: #q-factor means of much of cros-sec-area that each root gets
			r=sqrt(q)*Rin
			if self.specie=='pine':
				L=abs(random.normalvariate(0.510, sqrt(0.671))) #length of root
			elif self.specie=='spruce':
				L=abs(random.normalvariate(0.480, sqrt(0.549))) #length of root
			elif self.specie=='leaf':
				L=abs(random.normalvariate(0.510, sqrt(0.658))) #length of root
			if random.uniform(0,1)>0.13 and r>self.rLim and L>2*Rin: #not a sinker root
				zfin=zin-sin(degree)*L
				ok=False
				lim=15*2*pi/360. #15 degrees in radians
				while not ok:
					th2=random.normalvariate(thM,thS)	
					th2=th2*(2*round(random.uniform(0,1))-1) #switches between - and + with 50% prob
					ok=True
					for ang in angles:
						if abs(th2-ang)<lim:
							ok=False
				angles.append(th2) #we have a fine angle.
				rfin=r*(1-newtf) #r=rin-(rin-rfin)*x/L .. r at an arbitrary x length. 0<x<L
				direction=th+th2
				#set up the nodes:
				c1=cart([-r, 0], origin=origin, direction=direction, fromLocalCart=True)
				c2=cart([-rfin, L], origin=origin, direction=direction, fromLocalCart=True)
				c3=cart([rfin, L], origin=origin, direction=direction, fromLocalCart=True)
				c4=cart([r, 0], origin=origin, direction=direction, fromLocalCart=True)
				self._makeRoot(origin, direction, [c1,c2,c3,c4], r, rfin,L, zin, zfin, visible=first)
				end=cart([0, L], origin=origin, direction=direction, fromLocalCart=True)
				if rfin>self.rLim and zfin>-self.zlim:
					self._branch(end, rfin, direction, zfin) #splits again.
	def _makeRoot(self,pos, direction, nodes, rin, rfin, length, zin=0, zfin=0, visible=False):
		"""splits a root, if it is too big."""
		cart=getCartesian
		origin=pos
		[c1,c2,c3,c4]=nodes #c1,c4 are at the initial position.
		middle=cart([0, length/2.], origin=origin, direction=direction, fromLocalCart=True)
		rmiddle=rin-(rin-rfin)*0.5 #middle radius
		zmiddle=zfin-(zfin-zin)*0.5
		if length > Root.cap and not visible:
			c5=cart([-rmiddle, length/2.], origin=origin, direction=direction, fromLocalCart=True)
			c6=cart([rmiddle, length/2.], origin=origin, direction=direction, fromLocalCart=True)
			self._makeRoot(pos, direction, [c1,c5,c6,c4], rin, rmiddle, zin=zin, zfin=zmiddle,length=length/2.) #two new roots.
			if zmiddle>-self.zlim:
				self._makeRoot(middle, direction, [c5,c2,c3,c6], rin=rmiddle,rfin=rfin, zin=zmiddle, zfin=zfin, length=length/2.)
		else: #make a new root.
			Bradius=getDistance(middle, c1) #assumes that r>rfin
			root=Root(pos=middle, z=zmiddle, Bradius=Bradius, diameter=2*rmiddle, nodes=nodes, direction=direction, terrain=self.terrain, stump=self, visible=visible)
			self.roots.append(root)
   	def _numberOfSubRoots(self):
		"""handles a lot of random stuff, often correlated to the species."""
		r=random.uniform(0,1)
	  	if self.specie=='pine':
			zDegree=random.normalvariate(13,17.3)*pi/180. #vertical angle.
			prob3=0.09
			prob2=0.77
			q2=[0.70, 0.30]
			thM=39 #mean horizontal angle in degrees
			thS=46 #std of above
		elif self.specie=='spruce':
			zDegree=random.normalvariate(12,14.8)*pi/180.
			prob3=0.13
			prob2=0.78
			q2=[0.65,0.35]
			thM=32 #mean horizontal angle
			thS=37 #std of above
		elif self.specie=='leaf':
			zDegree=random.normalvariate(12,15.8)*pi/180.
			prob3=0.019
			prob2=0.70
			q2=[0.6,0.4]
			thM=31 #mean horizontal angle
			thS=39 #std of above
		thM=thM*pi/180
		thS=thS*pi/180
		if zDegree<=0: zDegree=2*pi/180.
		elif zDegree>pi/2.: zDegree=pi/2.
		if r<prob3:
			n=3
			tf=random.normalvariate(0.1,0.011)
			q=[0.5, 0.25, 0.25]
			if tf<0.089: tf=0.01
		elif r<prob2+prob3:
			n=2
			tf=random.normalvariate(0.096,0.0156)
			q=q2
			if tf<0.08: tf=0.096
		else:
			n=1
			tf=random.normalvariate(0.04,0.0206)# mm/dm root length.. insane definitions..
			q=[1]
			if tf<0.0194: tf=0.04		
		return [n, tf, q, zDegree, thM, thS]
	def draw(self, ax):
		"""draws stump root, starting from the nodes."""
		for r in self.roots:
			if not r.visible: r.draw(ax) #if visible, terrain draws root
		cir = Circle(tuple(self.pos), radius=self.radius, facecolor=self.color)
		ax.add_patch(cir)
