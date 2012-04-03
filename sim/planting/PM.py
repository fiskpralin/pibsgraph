#!/usr/bin/env python
from sim.tools import SimSeries
from sim.tools import SimExtend
import terrain
from plantMTerrain import PlantMTerrain
from plot import *
import machines
from excel_io import ExcelOutput
from machines.planting.plantmachine import PlantMachine

import time
import copy
import string
import numpy as np
from math import *

class PMSimSeries(SimSeries):
	"""
	Introduces the time for planting device refills. Should be inherited by all simulation series
	Information needed for the refill procedure is the time to refill and the number of plants refilled each time.
	we assume that mtype variable is standardized in the 1a1h/1a2h/2a2h/2a4h way.

	This class makes sure that there's order in the simulation series.

	In the master thesis, the data was handled before it was stored. Thus, simulation series where a parameter
	was varied could be stored in a single file. In the new file system every run is stored.
	If for example the crane maximum length is varied in the steps [9,10,11], three xls files are produced.
	These files have ~50 rows each, one for each simulation. Thus, a simulation series produces a folder of
	files. The name of the folder is the name of the class, e.g. "varyAutomation" followed by a time stamp.
	Some similar system will be applied to the files.

	A standard xls-template is used, as requested from SLU. It can be found in the same folder as this file and named template.xls. The output is placed in the "outputFiles"-folder.
	"""
	def __init__(self, G):
		SimSeries.__init__(self,G)
		self.reset()
	def reset(self):
		"""
		resets the simseries instance. Called when varying stuff and so on.
		"""
		self.excelOut=None
		self.cumTime=0
		self.cumTrees=0
		self.lastRefill=self.cumTrees #last refill was at cumTrees 0
		self.timeData=None
		self.sims=0
		self.bestCase=False
		self.worstCase=False
	def run(self):
		raise Exception('run must be implemented in PMSimSeries subclass.')
	def updateTimeData(self, s, reloaded=False, reloadtime=0):
		"""
		self.timeData should be a list with all the relevant information that will be stored.
		for simplicity, the list is in the same order as the excel document. This increases speed, but
		the drawback is that it slightly decreases the readability.
		Readability is provided by the comments below.

		For clarity, this is the data that is stored from a single simulation, e.g. productivity, queue-time etc.

		The index in the list corresponds to the column.
		"""
		self.timeData=[]
		t=self.timeData
		t.append(self.cumTime) #total time in s
		t.append(self.cumTrees) #total number of seedlings planted
		t.append(float(self.cumTrees)/(float(self.cumTime)/3600.0))
		t.append(self.sims)
		t.append(len(s.m.treesPlanted))
		t.append(s.stats['plant attempts'])
		t.append(s.stats['mound attempts'])
		t.append(s.stats['remound attempts'])
		t.append(s.stats['stumps in WA'])
		t.append(s.stats['stumps in WA sum diamater'])
		t.append(s.stats['immobile boulder struck'])
		t.append(s.stats['immobile vol sum']*1000) #change to dm3
		t.append(s.stats['number of dibble disr stones in mound'])
		t.append(s.stats['dibble distr stone vol cum']*1000) #change to dm3		
		t.append(s.now()-s.m.timeConsumption['machineMovement']) #total time at stationary point
		t.append(s.now()/max(1,float(len(s.m.treesPlanted))))
		t.append(s.m.timeConsumption['machineMovement'])
		if reloaded:
			reloaded=1
		else:
			reloaded=0
		t.append(reloaded)
		t.append(reloadtime)
		t.append(s.m.timeConsumption['searchTime'])
		t.append(s.m.timeConsumption['switchFocus'])
		t.append(s.stats['queue percent'])
		t.append(s.stats['work percent'])
		t.append(s.stats['work time'])
		t.append(s.stats['rest time'])
		for pDev in s.m.pDevs:
			for pH in pDev.plantHeads:
				t.append(pDev.timeConsumption['crane movement'])
				t.append(pH.timeConsumption['mounding'])
				t.append(pH.timeConsumption['planting'])
				t.append(pH.timeConsumption['halting'])
		# to be continued
	def _singleSim(self,G,mtype):
		"""
		this is a single simulation. Does some overhead stuff that is connected to the series
		and not to the single simulation. E.g. plant refill, some data storage..'

		Returns the simulation instance and an indicator 
		"""
		quitPossible=False
		SimExtend.seed='undefined'
		mtype=str(mtype)
		if mtype=='1a1h':
			headsCapacity=72
			refillTime=223
		elif mtype=='1a2h':
			headsCapacity=162
			refillTime=366
		elif mtype=='2a2h':
			headsCapacity=144
			refillTime=362
		elif mtype=='2a4h':
			headsCapacity=324
			refillTime=648
		else: raise Exception("does not recognize machine type %s"%(mtype,))
		s=PlantmSim(G,vis=False,mtype=mtype)
		self.cumTime+=s.now()
		self.cumTrees+=len(s.m.treesPlanted)
		t=0
		if self.cumTrees-self.lastRefill>headsCapacity:
			self.cumTime+=refillTime
			self.lastRefill=self.cumTrees
			quitPossible=True #we are allowed to exit..
		self.sims+=1
		if quitPossible: #reloaded
			self.updateTimeData(s, reloaded=True, reloadtime=refillTime)
		else:
			self.updateTimeData(s)
		self._storeData(s)
		return s, quitPossible
	def _storeData(self, s=None):
		"""
		stores the data. The strategy is to store it all, it has to be processed later.
		The purpose of this function is to standardize the column "design", so the numerous data files are easily
		accessed and generic scripts can be written for them.

		s has to be given if this is the first time the method is called and an output should  be created.
		"""
		e=self.excelOut
		if not e: #start a new instance.
			if not s:
				raise Exception('simulation instance has to be provided at first call for _storeData')
			e=ExcelOutput(template='sim/planting/template.py', out=self.filename)
			self.excelOut=e
			e.changeSheet(0)
			#set all the parameters
			paramRow=3
			type=copy.copy(s.m.type) #we need to shift 2aXh because of new directions...
			e.modify(paramRow, 0, s.m.type) #machine type
			e.modify(paramRow, 1, s.G.terrain.ttype) #terrain type
			e.modify(paramRow, 2, s.G.terrain.stumpFile) #terrain type
			e.modify(paramRow, 3, s.G.terrain.stumpsPerH) #stumps/ha
			e.modify(paramRow, 4, s.G.terrain.groundModel)
			e.modify(paramRow, 5, s.G.terrain.blockQuota*100) #percent
			e.modify(paramRow, 6, s.G.terrain.boulderFreq)
			e.modify(paramRow, 7, s.G.terrain.meanBoulderV*1000)
			e.modify(paramRow, 9, s.m.craneMaxL)
			if s.m.type[0:2]=='2a':
				e.modify(paramRow, 10, s.m.craneIntersect)
				e.modify(paramRow, 11, s.m.angleLim[0]*180/pi)
				e.modify(paramRow, 12, s.m.angleLim[1]*180/pi)
			if len(s.m.pDevs[0].plantHeads)==2: e.modify(paramRow, 13, s.m.pDevs[0].plantSepDist*100)
			e.modify(paramRow, 14, s.m.pDevs[0].plantHeads[0].width*100)
			e.modify(paramRow, 15, s.m.stockingRate)
			e.modify(paramRow, 16, s.m.plantMinDist)
			e.modify(paramRow, 17, s.m.times['switchFocus'])
			e.modify(paramRow, 18, str(s.G.automatic['automove']))
			e.modify(paramRow, 19, str(s.G.automatic['mound']))
			e.modify(paramRow, 20, s.m.velocities['radial'])			
			e.modify(paramRow, 21, str(self.bestCase))
			e.modify(paramRow, 22, str(self.worstCase))
		e.changeSheet(1) #the time-data sheet
		tdrow=2+self.sims
		for col,val in enumerate(self.timeData):
			print tdrow, col, val
			e.modify(tdrow,col,round(val,2)) #writes the time data. Request from BTE to round off
		e.save()
		return e
	def makeFolder(self,path=None):
		"""
		creates a folder
		"""
		if path:
			if not os.path.exists(path):
				os.makedirs(path)
			return path
		if self.outpfolder: return self.outpfolder
		self.outpfolder='outputFiles'+'/'+'%s_%s'%(self.__class__.__name__, string.join([string.strip(a) for a in string.split(time.ctime(time.time()))], "_"))
		if not os.path.exists(self.outpfolder):
			os.makedirs(self.outpfolder)
		return self.outpfolder

########################################
# plantingmachine simulation series
########################################

def varyAll(i=1):
	"""
	runs all the vary-classes for plantingmachine
	"""
	try:
		i=eval(i)
	except TypeError: pass #i is already integer, continue
	VaryTerrain(i)
	VaryCraneRadius(i)
	VaryAttachLoc(i)
	VaryDegrees(i)
	VaryDibbleDist(i)
	VaryMoundingbladeWidth(i)
	VaryTargetStockingrate(i)
	VaryMinDist(i)
	VaryOperatorSwitchFocusSkill(i)
	VaryAutomation(i)
	VaryRadialCraneSpeed(i)
	BestCase(i)
	WorstCase(i)
class VaryCraneRadius(PMSimSeries):
	"""this class describes a simulation that varies the automation level.
	tested: Yes
	"""
	def __init__(self,it=2, G=None, rList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		if rList==None:
			rList=[(4,9),(4,10),(4,11)]
		for cL in rList:
			for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+str(cL[0])+'_'+str(cL[1])+'.xls'
				G=copy.deepcopy(self.G)
				G.craneLim=list(cL)
				quitPossible=False
				i=0
				while i<it or (not quitPossible):
					G.terrain.restart() #makes the stumps non-static.. different distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d, it:%d quitpossible: %s"%(i,it, str(quitPossible))
				self.reset()

class VaryTerrain(PMSimSeries):
	"""
	this class describes a simulation that varies the automation level....
	tested: Yes
	"""
	def __init__(self,it=2,G=None):
		PMSimSeries.__init__(self,G)
		folder=self.makeFolder()
		tList=['5','4','3','2','1','0']
		for ttype in tList:
			for mtype in ['2a2h', '2a4h', '1a1h', '1a2h' ]:
				self.filename=folder+'/'+mtype+'_'+'ttype'+str(ttype)+'.xls'
				G=copy.deepcopy(self.G)
				G.terrain=PlantMTerrain(G=G, ttype=ttype)
				quitPossible=False
				i=0
				while i<it or (not quitPossible):
					G.terrain.restart()
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class VaryAttachLoc(PMSimSeries):
	"""
	this class describes a simulation that varies the automation level.
	tested: Yes
	"""
	def __init__(self,it=2,G=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		pList=[3+i for i in range(3)]
		for p in pList:
			for mtype in ['2a2h', '2a4h']: #only 2a relevant
				G=copy.deepcopy(self.G)
				G.PMattach=p
				self.filename=folder+'/'+mtype+'_'+'attach'+'_'+str(p)+'.xls'
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart()
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class VaryDegrees(PMSimSeries):
	"""
	this class describes a simulation that varies the allowed angle for the 2a cranes.
	tested: Yes
	"""
	def __init__(self,it=2,G=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		dList=[(0, 45), (0,60), (-10,45), (-20, 45)]
		for dmin,dmax in dList:
			for mtype in ['2a2h', '2a4h']: #only 2a relevant
				G=copy.deepcopy(self.G)
				G.PMangleLim=[dmin*pi/180.0, dmax*pi/180.0]
				self.filename=folder+'/'+mtype+'_'+'angles'+'_'+str(dmin)+'_'+str(dmax)+'.xls'
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different root distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
				
class VaryDibbleDist(PMSimSeries):
	"""this class describes a simulation that varies the distance between the planting dibbles
	tested: Yes
	"""
	def __init__(self,it=2, G=None, dList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		if not dList: dList=[2.01, 1.50, 1.00] #default
		for d in dList:
			for mtype in ['2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+'_'+str(d)+'.xls'
				G=copy.deepcopy(self.G)
				G.PMplantSepDist=d #hmm.. check so that plantmindist doesn't interfere..
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class VaryMoundingbladeWidth(PMSimSeries):
	"""this class describes a simulation that varies the width of the mounding blade
	tested: Yes (kind of..)
	"""
	def __init__(self,it=2, G=None, wList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		if not wList: wList=[0.45, 0.40, 0.60] #default
		for w in wList:
			for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+'_'+str(w)+'.xls'
				G=copy.deepcopy(self.G)
				G.PMbladeWidth=w #hmm.. check so that plantmindist doesn't interfere..
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class VaryTargetStockingrate(PMSimSeries):
	"""this class describes a simulation that varies the stockingrate
	tested: Yes
	"""
	def __init__(self,it=2, G=None, wList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		if not wList: wList=[1500, 2000, 2500] #default
		for w in wList:
			for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+'_'+str(w)+'.xls'
				G=copy.deepcopy(self.G)
				G.PMstockingRate=w
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class VaryMinDist(PMSimSeries):
	"""this class describes a simulation that varies the stockingrate
	tested: Yes
	"""
	def __init__(self,it=2, G=None, wList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		if not wList: wList=[1.0, 1.5] #default
		for w in wList:
			for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+'_'+str(w)+'.xls'
				G=copy.deepcopy(self.G)
				G.plantMinDist=w 
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class VaryOperatorSwitchFocusSkill(PMSimSeries):
	"""this class describes a simulation that varies the operator focus switching skill
	tested: Yes
	"""
	def __init__(self,it=2, G=None, wList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		if not wList: wList=[0,1,3] #default
		for w in wList:
			for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+'_'+str(w)+'.xls'
				G=copy.deepcopy(self.G)
				G.PMfocusSwich=w 
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
		
class VaryAutomation(PMSimSeries):
	"""this class describes a simulation that varies the operator focus swiching skill
	tested: Yes
	"""
	def __init__(self,it=2, G=None, wList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G) #terrain type 4, many stones many stumps
		folder=self.makeFolder()
		default={'mound': False, 'plant': True, 'automove': False, 'micrositeSelection': False, 'moveToMicro': False,'haltMound':False, 'clearForOtherHead':True}
		if not wList: wList=[(False, False), (True, False), (False, True), (True, True)] #default
		for move, mound in wList:
			for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+'_'+'move:'+str(move)+'_mound:'+str(mound)+'.xls'
				G=copy.deepcopy(self.G)
				G.automatic=default
				G.automatic['mound']=mound
				G.automatic['automove']=move
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different distributions every time.
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class VaryRadialCraneSpeed(PMSimSeries):
	"""this class describes a simulation that varies the operator focus swiching skill
	tested: Yes
	"""
	def __init__(self,it=2, G=None, wList=None):
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		if not wList: wList=[0.8,1.6,3.2]
		for w in wList:
			for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
				self.filename=folder+'/'+mtype+'_'+str(w)+'.xls'
				G=copy.deepcopy(self.G)
				G.PMradialCraneSpeed=w
				quitPossible=False
				i=0
				while i<it or not quitPossible:
					G.terrain.restart() #makes the stumps non-static.. different distributions every time
					s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
					i+=1
					print "No of sims: %d"%(i)
				self.reset()
class BestCase(PMSimSeries):
	"""this class describes a simulation that varies the operator focus switching skill
	tested: Yes
	"""
	def __init__(self,it=2, G=None):
		self.bestCase=True
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		#self.G.PMattach=3
		self.G.PMangleLim=[-20*pi/180, 45*pi/180]
		self.G.PMplantSepDist=1.00
		self.G.PMbladeWidth=0.40
		for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
			self.filename=folder+'/'+mtype+'_'+'bestCase'+'.xls'
			G=copy.deepcopy(self.G)
			quitPossible=False
			i=0
			while i<it or not quitPossible:
				G.terrain.restart() #makes the stumps non-static.. different distributions every time.
				s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
				i+=1
				print "No of sims: %d"%(i)
			self.reset()
class WorstCase(PMSimSeries):
	"""this class describes a simulation that varies the operator focus swiching skill
	tested: Yes
	"""
	def __init__(self,it=2, G=None):
		self.worstCase=True
		PMSimSeries.__init__(self,G)
		if not G:
			self.G.terrain=PlantMTerrain(G=self.G)
		folder=self.makeFolder()
		#self.G.PMattach=6
		self.G.PMangleLim=[0, 45*pi/180]
		self.G.PMplantSepDist=2.01
		self.G.PMbladeWidth=0.60
		for mtype in ['1a1h', '1a2h', '2a2h', '2a4h']:
			self.filename=folder+'/'+mtype+'_'+'worstCase'+'.xls'
			G=copy.deepcopy(self.G)
			quitPossible=False
			i=0
			while i<it or not quitPossible:
				G.terrain.restart() #makes the stumps non-static.. different distributions every time.
				s, quitPossible=self._singleSim(G,mtype) #G is copied later, so does not affect G
				i+=1
				print "No of sims: %d"%(i)
			self.reset()
def findBugs():
	"""
	just a thing for the planting simulation code
	"""
	import sim.planting.PM as PM
	while True:
		G=globalVar()
		seed=int(random.uniform(1,100000))
		seed=39971
		random.seed(seed)
		G.seed=seed
		"""r=random.uniform(4,5)
		R=random.uniform(9,12)
		G.craneLim=[r,R]"""
		mtype='2a' #only interesting
		#G.terrain=PlantMTerrain(G=G)
		G.PMattach=random.choice([3,4,5])
		#G.plotDelay=15
		print "attach:", G.PMattach
		"""
		G.automatic={}
		G.automatic['mound']=random.choice([True, False])
		G.automatic['plant']=random.choice([True, False])
		G.automatic['automove']=random.choice([True, False])
		G.automatic['micrositeSelection']=random.choice([True, False])
		G.automatic['moveToMicro']=random.choice([True, False])
		G.automatic['haltMound']=random.choice([True, False])
		G.automatic['clearForOtherHead']=random.choice([True, False])
		print G.automatic, G.craneLim"""
		s=PM.PlantmSim(mtype=mtype,G=G, vis=False) #G is copied later, so does not affect G
		if s.productivity<50 or s.now()>1000:
			s.p=plot.PlotWorld('ds', sim=s, delay=1e10,G=s.G)
			s.p.update()
			import matplotlib.pyplot as plt
			plt.show()
			print G.automatic, G.craneLim, G.terrain.ttype, "switching:", s.m.timeConsumption['switchFocus']
			raw_input('plantmachine seems to be stuck..Check it..')
			raise Exception('plantmachine seems to be stuck..Check it..')
	

#########################################
#plantmSim - a single simulation
 ##########################################
class PlantmSim(SimExtend):
	"""
	class for a single simulation with a 1a or 2a plantmachine
	286"""
	def __init__(self, G=None, vis=True, anim=False, mtype='2a', ttype='random'):
		SimExtend.__init__(self, G, vis, anim, animDelay=0.3) #animDelay==standard value for animations
		self.stats={'plant attempts':0, 'mound attempts':0, 'remound attempts':0, 'stumps in WA':None, 'stumps in WA sum diameter':0, 'immobile boulder struck':0, 'immobile vol sum':0, 'number of dibble disr stones in mound':0, 'dibble distr stone vol cum':0, 'queue percent':0,'work percent':0, 'work time':0,'rest time':0  }
		if not self.G.terrain:
			self.G.terrain=PlantMTerrain(G=self.G, ttype=ttype)
		tic=time.clock()
		if self.G.automatic=='undefined':
			if mtype[0:2]=='1a':
				self.G.automatic={'mound': False, 'plant': True, 'automove': False, 'micrositeSelection': False, 'moveToMicro': False,'haltMound':False, 'clearForOtherHead': False}
			else:
				self.G.automatic={'mound': True, 'plant': True, 'automove': True, 'micrositeSelection': False, 'moveToMicro': False, 'haltMound':True, 'clearForOtherHead': True}
		self.m=PlantMachine(name="planter", sim=self, G=self.G, mtype=mtype)
		self.activate(self.m,self.m.run())
		self.simulate(until=self.G.maxtime)
		#postprocessing
		self.productivity=len(self.m.treesPlanted)/(self.now()/3600.) #trees/hour
		print "result: %d trees in %d time, productivity: %f trees/hour"%(len(self.m.treesPlanted), self.now(), self.productivity)		
		self.setQueuePerc()
		print "stumps in WA sum diameter:", self.stats['stumps in WA sum diamater']
		print "number of remounds", self.stats['remound attempts']
		if self.p: #if we have a plotter instance
			self.p.terminate()
			plt.show() #nothing is processed in paralell with this, thus last:
	def plotMoni(self, ax, number):
		"""
		defines the monitors that should be plotted.
		assumes that matplotlib axis ax and number \in {1,2,3} is given.
		"""
		if number==1: #plot driver activity
			driver=self.m.driver
			moni=driver.actMon
			if len(moni)>0:
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([-0.1, self.now(), -0.1, 1.2])
				ax.grid(True)
				ax.set_ylabel('Working pattern')
				ax.set_yticks((0,  1))#, ('idle', 'busy'))#, color = 'k')
		elif number==2: #plot driver waiting queue
			moni=self.m.driver.waitMon
			if len(moni)>0:
				maxt=int(max(moni.yseries()))
				ticks=[]
				for i in range(maxt+1): ticks.append(i)
				ax.set_yticks(tuple(ticks))
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([-0.1, self.now(), -0.1, max(moni.yseries())+0.1])
				ax.grid(True)
				ax.set_ylabel('Wait queue for driver.')
			"""moni=self.m.driver.waitMon
			if len(moni)>0:
				axt=int(max(moni.yseries()))
				ticks=[]
				for i in range(maxt+1): ticks.append(i)
				ax.set_yticks(tuple(ticks))
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([-0.1, m.sim.now(), -0.1, max(moni.yseries())+0.1])
				ax.grid(True)
				ax.set_ylabel('Wait queue for driver.')	"""
		elif number==3: #plot trees planted
			moni=self.m.treeMoni
			#trees planted
			if len(moni)>0:
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([-0.1, self.now(), -0.1, (max(moni.yseries())+1)])
				ax.grid(True)
				ax.set_xlabel('time(s)')
				ax.set_ylabel('trees planted')
		return ax
				
