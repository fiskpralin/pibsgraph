#!/usr/bin/env python
from SimPy.Simulation  import *

class Observer(Process):
	"""
	Class that observes the heads and their respective activities
	and driver usages such as to make statistics of it.
	"""
	def __init__(self, name, sim, G):
		Process.__init__(self, name, sim)
		self.m=self.sim.m
		self.G=G
		self.tstep=0.1
		print 'Successfully initiated an Observer called', self.name
		for head in self.m.heads:
			if head=='left':
				#Activity monitor
				self.lAMoni=Monitor(name='leftyActMon')
				print 'Made a monitor named', self.lAMoni.name
				
				#Waiting for bundler monitor
				self.lWBMoni=Monitor(name='leftyWaitForBundlerMon')
				print 'Made a monitor named', self.lWBMoni.name

				#Waiting for driver monitor
				self.lWDMoni=Monitor(name='leftyWaitForDriverMon')
				print 'Made a monitor named', self.lWDMoni.name
				
			if head=='right':
				#Activity monitor
				self.rAMoni=Monitor(name='rightyActMon')
				print 'Made a monitor named', self.rAMoni.name

				#Waiting for bundler monitor
				self.rWBMoni=Monitor(name='rightyWaitForBundlerMon')
				print 'Made a monitor named', self.rWBMoni.name

				#Waiting for driver monitor
				self.rWDMoni=Monitor(name='rightyWaitForDriverMon')
				print 'Made a monitor named', self.rWDMoni.name

		self.bundlerActiveMoni=Monitor(name='bundlerActMon')#this is for debug but can maybe be used?

	def run(self):
		self.update()
		while True:
			yield hold, self, self.tstep
			self.update()

	def update(self):
		if self.m.hasBundler and self.m.bundler.active()==True: bundlerActive=1
		else: bundlerActive=0
		if self.m.heads['left'].active()==True: leftActive=1
		else: leftActive=0
		if self.m.heads['left'].queuing(self.m.driver)==True: waitingD=1
		else: waitingD=0
		
		self.bundlerActiveMoni.observe(bundlerActive, self.sim.now())
		self.lAMoni.observe(leftActive, self.sim.now())
		self.lWDMoni.observe(self.m.heads['left'].queuing(self.m.driver), self.sim.now())
		self.lWBMoni.observe(self.m.heads['left'].waitingForBundler, self.sim.now())

		if leftActive==1 and waitingD==1: raise Exception('Some error in active-waiting interaction of left head')
		
		if self.m.nCranes==2:
			if self.m.heads['right'].queuing(self.m.driver)==True: waitingD=1
			else: waitingD=0
			if self.m.heads['right'].active()==True: rightActive=1
			else: rightActive=0
			self.rAMoni.observe(rightActive, self.sim.now())
			self.rWDMoni.observe(waitingD, self.sim.now())
			self.rWBMoni.observe(self.m.heads['right'].waitingForBundler, self.sim.now())
			if rightActive==1 and waitingD==1: raise Exception('Some error in active-waiting interaction of right head')
		

