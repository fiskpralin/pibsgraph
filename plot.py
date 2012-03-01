#!/usr/bin/env python
import matplotlib as mpl
import matplotlib.pyplot as plt
import pylab
import os, sys, shutil
import functions as fun
from SimPy.Simulation  import *

class PlotWorld(Process):
	"""
	Implemented as a process for the ability to plot at a fps. Does not interact with the
	simulation, only gathers information and prints it.
	Needs to be terminated for animations to work properly
	supports three monitors that has to be specified in simulation method plotMoni. Otherwise plots world
	"""
	def __init__(self,name, sim, delay=None, G=None):
		Process.__init__(self, name, sim)
		self.G=G

		if delay:
			self.tstep=delay
		elif not G: #two ways to give delay info. 
			raise Exception('PlotWorld needs information about the delay..')
		else:
			self.tstep=G.plotDelay #this stuff should already have been handled in simextend, but we take care of it anyway.
		if not self.sim.anim and self.tstep < 1e5: #default is like 1e10..
			plt.ion()
		self.fig = plt.figure()
		#look for monitors.
		try:
			self.sim.plotMoni() #will throw an exception if specified since arguments are not given.
			monImpl=False
		except TypeError:
			monImpl=True		
		if self.G.noMonitors or (not monImpl):
			self.ax1 = self.fig.add_subplot(111,aspect='equal', axisbg='#A2CD5A')
			self.ax2=None #not implemented in Simulation. See tools.simextend for instructions.
			self.ax3=None
			self.ax4=None
		else:
			self.ax1 = self.fig.add_subplot(121,aspect='equal', axisbg='#A2CD5A')
			self.ax2 = self.fig.add_subplot(322)
			self.ax3 = self.fig.add_subplot(324)
			self.ax4 = self.fig.add_subplot(326)
		if self.sim.anim:
			if not os.path.exists('animtemp'):
				os.makedirs('animtemp')
	def run(self):
		while True:
			yield hold, self, self.tstep
			if not self.sim.anim:
				self.update()
				plt.draw()
				raw_input("Press return key to continue...")
			else:
				self.update()
	def update_world(self,ax1, lim=None):
		"""
		updates the animation, i.e terrain, machine etc visualization
		Usually called internally by update()
		"""
		self.G.terrain.draw(ax1)
		if self.G.roadNet:
			for r in self.G.roadNet.edges(data=True):
				r[2]['r'].draw(ax1)
		#plot machines/plantingheads
		#determine order: pheads first, to avoid overlapping.
		mOrdered=[]
		for mtmp in self.sim.machines:
			if mtmp.__class__.__name__=='Machine':
				mOrdered.append(mtmp) #last
			else:
				mOrdered.insert(0,mtmp)#first
		for mtmp in mOrdered:
			mtmp.draw(ax1)
			if mtmp.__class__.__name__=='Machine':
				m=mtmp
		if lim:
			ax1.axis(lim)
		elif  mtmp.__class__.__name__=='PlantMachine' and self.sim.anim: ax1.axis([mtmp.pos[0]-mtmp.craneMaxL-1,mtmp.pos[0]+mtmp.craneMaxL+1, mtmp.pos[1]-mtmp.craneMaxL-1, mtmp.pos[1]+mtmp.craneMaxL+1 ])
		else: ax1.axis(self.G.terrain.xlim+self.G.terrain.ylim)
		ax1.set_xlabel('x (m)')
		ax1.set_ylabel('y (m)')	
	def update(self):
		print self.sim.now(), ",starts plotting procedure"
		G=self.G
		fig=self.fig
		p='undefined'
		fun.cleanMonitors(self.sim)
		for ax in [self.ax1,self.ax2,self.ax3,self.ax4]:
			if ax: ax.clear()
		#plot terrain
		self.update_world(self.ax1)
		#plot monitors:
		if self.ax2: #assumes that 3&4 are also given, but works anyway
			self.ax2=self.sim.plotMoni(self.ax2, 1)
			self.ax3=self.sim.plotMoni(self.ax3, 2)
			self.ax4=self.sim.plotMoni(self.ax4, 3)
		#show
		if self.sim.anim:
			fig.savefig('animtemp/_tmp%06d.png'%round(self.sim.now()))
		else:
			plt.draw() #not really needed, I think? done in run
	def terminate(self):
		"""
		This method should be called when done with anim.
		"""
		self.update()
		fun.cleanMonitors(self.sim)
		if self.sim.anim:
			os.system("mencoder 'mf://animtemp//_tmp*.png' -mf type=png:fps=10 -ovc lavc -lavcopts vcodec=wmv2 -oac copy -o animation.mpg") #only works if mencoder is installed and on linux.
			#remove files:
			if os.path.exists('animtemp'):
				shutil.rmtree('animtemp')

	
