#!/usr/bin/env python
from SimPy.Simulation  import *

class Observer(Process):
	"""
	Class that observes the heads and their respective activities
	and driver usages such as to make statistics of it.
	"""
	def __init__(self, name, sim, G):
		Process.__init__(self, name, sim)
		self.G=G
		self.tstep=0.5
		print 'Successfully initiated an Observer called self.o'
		#self.active=0#0 or 1 to indicate if head is working or not
		#self.activeMoni=Monitor(name=self.side+'WorkOrWait')
		#self.activeMoni.observe(self.active)

	def run(self):
		while True:
			yield hold, self, self.tstep
			self.update()

	def update(self):
		pass

