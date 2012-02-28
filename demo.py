from sim.tools import globalVar, SimExtend #a global veriable and simextend class that does a lot of overhead stuff.
import terrain
from terrain.terrain import Terrain, Tree
from plot import * #does all the plotting
import machines
from machines.basics import Machine #this class is common for all machines
from functions import *
import random
from collision import collide
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np
from math import *
#this is a minimal simulation. 

class DumbMachine(Machine):
	"""
	a random walking machine
	"""
	def __init__(self, sim, G, color):
		Machine.__init__(self, sim=sim, name='dumbass', G=G)
		self.velocities['machine']=3 #m/s, default name, has to be there
		self.setPos([15,0])#Sets the initial position to random in the x direction and 0 in y
		self.IQ=10 #as I said... no AI here
		self.color=color
		self.trees=[]
	def run(self):
		"""
		the process execution method
		"""
		self.positions=[self.pos] #save, will later be plotted
		t=self.G.terrain #used to make code shorter and clearer
		while self.pos[1]<t.ylim[1]: #while machine is inside the domain
			newX=self.pos[0]
			newY=self.pos[1]+random.uniform(1,5)
			temp_treelist=t.GetTrees(self.pos,10)
			for a in temp_treelist:
				if a.harvested==False:
					self.trees.append(a)
					yield hold,self, 30#the time it takes to chop the tree (half a minute!)
					a.harvested=True
					a.isSpherical=False
					a.nodes=[[a.pos[0]-6,a.pos[1]-0.1],[a.pos[0]-6,a.pos[1]+0.1],[a.pos[0],a.pos[1]+0.1],[a.pos[0],a.pos[1]-0.1]]
			time=self.setPos([newX, newY])#the time it takes to move to the new place
			self.positions.append(self.pos)
			yield hold, self, time #waits
		"""Now all the trees should be harvested and so I tell the machine to go fetch them!"""
		for a in self.trees:
			newX=a.pos[0]
			newY=a.pos[1]
			time=self.setPos([newX,newY])
			self.positions.append([newX,newY])
			print "Moving out again to the harvested trees to pick them up"
			yield hold, self, time
		newPos=self.positions[0]
		self.positions.append(newPos)
		time=self.setPos(newPos)
		print "Moving home with all the trees which I have picked up in a stupid random order"
		yield hold, self, time
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
		self.corners=[]
		dx=1 #halfwidth of the machine
		dy=2 #halflength of the machine
		self.corners.append([+dx, +dy])
		self.corners.append([-dx, +dy])
		self.corners.append([-dx, -dy])
		self.corners.append([+dx, -dy])
		p=Polygon(np.array(self.makedirectional()), closed=True, facecolor=self.color)#here was the np.array(self.corners) command before
		ax.add_patch(p)

	def makedirectional(self):
		"""
		makes new positions of the nodes of the machine as to draw it in the correct direction
		honestly i don't think anythin is correct in this 'attempt'. stupid-
		"""
		nc=[[],[],[],[]]
		i=0
		for a in self.corners:
			x=a[0]
			y=a[1]
			r=sqrt(x*x+y*y)
			dposx=self.positions[-2][0]-self.positions[-1][0]
			dposy=self.positions[-2][1]-self.positions[-1][1]
			direction=atan(dposx/(dposy+0.0001))
			theta=atan(x/(y+0.0001))
			if y<0: theta=theta+pi
			nc[i].append(r*sin(theta+direction)+self.pos[0])
			nc[i].append(r*cos(theta+direction)+self.pos[1])
			i=i+1	
		return nc


class TheSimulation(SimExtend):
	"""
	specifies the simulation
	"""
	def __init__(self):
		G=globalVar() #create a global variable. 
		SimExtend.__init__(self,G=G)
		G.xlim=[0, 40]#the borders.
		G.ylim=[0,40] 
		self.G.terrain=Terrain(G=self.G)
		self.tree=Tree(pos=[20,25], radius=0.5, specie='pine', terrain=self.G.terrain)
		m1=DumbMachine(sim=self, G=G, color='b') #color is definitely the most important thing here.
		m1.velocities['machine']=0.5
		G.plotDelay=20
		self.p=PlotWorld(name='plotter',sim=self, G=G) #create a plotter instance...
		
		for m in [m1]:
			self.activate(m, m.run()) #activate machines
		self.activate(self.p, self.p.run())#activate some plotting thing i dont fully understand
		self.simulate(until=1e10)
		self.p.update() #abort p.
		print "simulation done"
		plt.show()

if __name__=='__main__':
	s=TheSimulation()
	
