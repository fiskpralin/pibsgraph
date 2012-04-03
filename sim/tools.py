#!/usr/bin/env python
from SimPy.Simulation  import *
from math import *

import terrain
import machines
import plot
import random
import copy
import os
import time
import numpy as np
from string import *

class globalVar():
	"""
	The global variables used in the simulations.
	Variables are stored as object variables and not class variables, to allow paralell simulations.
	fix: structure. How do we divide it into different type of simulations?
	"""
	def __init__(self): 
		self.areaPoly=None #defines the area by [(x,y)] coordinates. min three
		self.maxtime=1e10
		self.debug=True 
		self.terrain=None
		self.anim=False #animation off by defauklt
		self.plotDelay=1e10 #default... means that plotting is only done in the end.
		self.noMonitors=False #if true, only world is plotted
		self.productivity='undefined'
		self.automatic='undefined'
		self.plantMinDist=1.5
		self.PMstockingRate=None 
		self.PMplantSepDist=2.01 #standard separation distance for planting devices.
		self.PMbladeWidth=None
		self.PMangleLim=[0, 45.*2.*pi/360.]
		self.PMfocusSwitch=None
		self.PMattach=None
		self.PMradialCraneSpeed=None
		self.roadNet=None
		self.craneLim=[4,9]
		self.gridL=None
		self.seed=None
		self.simParam={} #stuff that cannot be fit anywhere else.

############################
# simSeries
###########################
class SimSeries():
	"""
	This class is intended to be the superclass for all simulation series. When a specific simulation is setup, one creates an instance of this class and runs it.
	"""
	def __init__(self, G):
		self.outpfolder=None
		if G is None:
			self.G=globalVar()
		self.filename=None #makes it possible to specify the filenmae before calling saveTofile.
		self.f=None #file, will later be initialized

	def run(self):
		pass

	def postProcess(self):
		pass

	def _saveToFile(self, string):
		"""
		generic method to save a string to the .txt file specified by self.filename.
		Does some common stuff first time and has a last resort for filename if not given
		Just remember to close the file.. self.g.close()
		"""
		#store to file
		string=str(string)
		if not self.f: #first time
			if not self.outpfolder: self.outpfolder='outputFiles/'+self.__class__.__name__
			if not os.path.exists(self.outpfolder):
				os.makedirs(self.outpfolder)
			if not self.filename:
				self.filename=self.outpfolder+'/'+'%s_%s'%(self.__class__.__name__, join([strip(a) for a in split(time.ctime(time.time()))], "_"))+".txt"
			self.f=open(self.filename, 'w')
			self.f.write(self.filename+'\n')
		if string[-1] != '\n': string+='\n'
		self.f.write(string)

class SimExtend(Simulation):
	"""
	Adds some common stuff. Also handles the case of not implementing plotMoni
	"""
	def __init__(self, G, vis=True, anim=False, animDelay=1, series=None):
		if not G: G=globalVar()
		self.G=G
		if G.seed: self.seed=G.seed
		else:
			self.seed=int(random.uniform(0,1000000)) #used to always be able to re-try an invalid simulation
		print "seed: ", self.seed
		random.seed(self.seed)
		self.machines=[] #all machines are put here in their Machine.__init__ method
		self.initialize() #must be there says SimPy standards
		self.stats={} #preferably, store all stats here..
		self.series=series #we might support references to the simseries, if given
		self._generalStatistics={} #..the idea is to automatically save all this data.
		self._generalStatisticsUnits={} #should have the same keys as the one above but with units
		self.anim=anim
		self.vis=vis
		if self.vis or self.anim:
			delay=G.plotDelay #standard, but handle separately below.
			if self.anim: #more fps
				self.vis=True #always..
				if delay>1e5: #then override it.
					delay=animDelay #standard for this simulation...given by the call.
			self.p=plot.PlotWorld(name='plotter',sim=self, delay=delay, G=self.G)
			self.activate(self.p,self.p.run()) #should be terminated to work properly. 
		else:
			self.p=None #means we plot nothing at all.
	def plotMoni(self, ax=None, number=None):
		"""
		PlotWorld invokes this function for each monitor. There are three monitors available for the simulation.
		Inherit and override if you want to use those monitors.
		"""
		return None #means that we won't use monitors.
	def setQueuePerc(self):
		"""
		returns the percent of time that the ques is nonempty
		"""
		try: mon=self.m.driver.waitMon
		except:
			print "simulation does not have a m.driver.waitMon. Cannot calculate QueuePerc."
			return False
		if len(mon)<2:
			self.queuePerc=0
		else:
			tTot=self.now()
			tQ=0
			tNQ=0 #non-queue time
			last=[0,0] #time 0 idle
			Q= mon[0]>=1 #queue right now?
			for el in mon:
				t=el[0]-last[0]
				if Q: tQ+=t
				else: tNQ+=t
				Q=el[1]>=1
				last=el
			if tQ+tNQ -tTot >0.01: raise Exception('something is wrong with the queueing perent/monitor reading.')
			self.queuePerc=100*tQ/tTot
		try: mon=self.m.driver.actMon
		except:
			print "simulation does not have a m.driver.actMon. Cannot calculate QueuePerc."
			return False
		if len(mon)<2:
			workPerc=-1
		else:	
			tTot=self.now()
			tQ=0.0
			tNQ=0.0 #non-queue time
			last=[0,0] #time 0 idle
			Q= mon[0]>=1 #working right now?
			for el in mon:
				t=el[0]-last[0]
				if Q: tQ+=t
				else: tNQ+=t
				Q=el[1]>=1
				last=el
			workPerc=100*tQ/tTot
		if self.queuePerc<0.00001: self.queuePerc=0	
		self.stats['queue percent']=self.queuePerc
		self.stats['work percent']=workPerc
		self.stats['work time']=workPerc*0.01*self.now()
		self.stats['rest time']=self.now()-self.stats['work time']
		print "queue percentage:", self.queuePerc, "work percentage:", workPerc
	def saveGeneralStats(self, name=None, value=None, unit=None):
		"""
		saves general statistics, that should be automatically written to output files
		"""
		if name is None or value is None or unit is None:
			print name, value, unit
			raise Exception('needs name, value and unit for general statistics to be saved')
		self._generalStatistics[name]=value
		self._generalStatisticsUnits[name]=unit
	def getGeneralStatsString(self):
		"""
		returns a string of the general stats:

		name \t value \t unit

		convenient for writing to output file.
		assumes that a tab is 4 char long.
		"""
		if len(self._generalStatistics.keys())==0: return " "
		tabl=4.0
		#first get some info about the width of the columns
		longest=[1,1]
		for key in self._generalStatistics.keys():
			if len(str(key))>longest[0]: longest[0]=len(str(key))
		for value in self._generalStatistics.values():
			if len(str(value))>longest[1]: longest[1]=len(str(value))
		longest[0]=round(longest[0]/tabl+1.0) #number of tabs
		longest[1]=round(longest[1]/tabl+1.0) 
		t1=int(ceil(longest[0]-4/tabl))
		t2=int(ceil(longest[1]-5/tabl))
		out="stat"+ "\t"*t1 +"value"+ "\t"*t2 +"unit\n"
		out+="-"*int(tabl*(longest[0]+longest[1]+1))+"\n"
		for key in self._generalStatistics.keys():
			s1=str(key)
			t1=longest[0]-floor(len(s1)/tabl)
			out+=s1+'\t'*int(t1)
			s2=str(self._generalStatistics[key])
			t2=longest[1]-floor(len(s2)/tabl)
			out+=s2+'\t'*int(t2)
			out+=str(self._generalStatisticsUnits[key])+'\n'
		return out
