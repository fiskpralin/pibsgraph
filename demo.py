from sim.tools import globalVar,SimExtend # a global veriable and simextend cladd that does a lot of overhead stuff.
import terrain
from terrain.terrain import Terrain, Tree
from plot import * #does all the plotting
import machines
from machines.basics import Machine #this class is common for all machines
from functions import *
import random

import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

#this is a minimal simulation. 

class DumbMachine(Machine):
	"""
	a random walking machine
	"""
	def __init__(self, sim, G, color):
		Machine.__init__(self, sim=sim, name='dumbass', G=G)
		self.velocities['machine']=3 #m/s, default name, has to be there
		self.setPos([1,1])
		self.IQ=10 #as I said... no AI here
		self.color=color
	def run(self):
		"""
		the process execution method
		"""
		self.positions=[self.pos] #save, will later be plotted
		t=self.G.terrain #shorter, clearer code
		while self.sim.now()<1000: #while time of the simulation is below 1000
			newX=random.uniform(t.xlim[0], t.xlim[1])
			newY=random.uniform(*tuple(t.ylim)) #another way to do the same thing.
			time=self.setPos([newX, newY]) #we don't check for collission with other machines and the tree.. a flaw..
			if skörda:
				tree.isSpherical=False
				tree.pos=[10,10]
				tree.nodes=[()]
			self.positions.append(self.pos)
			yield hold, self, time #waits
		self.sim.stopSimulation()
	def draw(self,ax):
		"""
		draws machine. This class has to have the above name and input
		"""
		x=[]
		y=[]
		for pos in self.positions:
			x.append(pos[0])
			y.append(pos[1])
		ax.plot(x,y, self.color)
		corners=[]
		dxdy=1 #create a square..
		corners.append([self.pos[0]+dxdy, self.pos[1]+dxdy])
		corners.append([self.pos[0]-dxdy, self.pos[1]+dxdy])
		corners.append([self.pos[0]-dxdy, self.pos[1]-dxdy])
		corners.append([self.pos[0]+dxdy, self.pos[1]-dxdy])		
		p=Polygon(np.array(corners), closed=True, facecolor=self.color)
		ax.add_patch(p)
class TheSimulation(SimExtend):
	"""
	specifies the simulation
	"""
	def __init__(self):
		G=globalVar() #create a global variable.
		G.plotDelay=5 #we will plot each 5th second. If you want this, do it before SimExtend.__init__
		SimExtend.__init__(self,G=G, vis=True) #does a lot of overhead stuff. can be found in sim.tools.py
		self.G.areaPoly=[(0,0), (40,0), (40,40), (0,40)] #specifies our world borders.
		self.G.terrain=Terrain(G=self.G)
		t=Tree(pos=[3,4], radius=0.5, specie='pine', terrain=self.G.terrain)
		m1=DumbMachine(sim=self, G=G, color='b') #color is definitely the most important thing here.
		m2=DumbMachine(sim=self, G=G, color='r') #sim=self.. self is here refering to TheSimulation
		for m in [m1,m2]:
			self.activate(m, m.run()) #activate machines
		self.activate(self.p,self.p.run())
		self.simulate(until=1e10)
		if self.p: #means that we have a plotting instance. All done in SimExtend.
			self.p.update() #abort p.
			print "simulation done"
			plt.show()

if __name__=='__main__':
	s=TheSimulation()
	
