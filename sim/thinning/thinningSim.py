from sim.tools import SimSeries, SimExtend, globalVar

from terrain.terrain import Terrain
#import machines
from machines.thinning.thMachine import ThinningMachine
from machines.thinning.thRoad import ThinningRoad
import functions as fun
import plot

import numpy as np
import matplotlib.pyplot as plt
import copy
from math import *
from string import *
import time
import random

class tryDiffConfigThinningMachine(SimSeries):
	"""
	This class will make simulations of the different kinds of machine configurations that
	we are interested in, in Mattias project for Dan, Urban and Ola spring 2012
	"""	
	def __init__(self,it=1):
		self.G=globalVar()
		#self.G.plotDelay=500 #for debug only
		self.G.areaPoly=[(0,0), (25,0), (25,40), (0,40)]
		self.G.terrain=Terrain(G=self.G)
		for treeFile in self.G.terrain.thinningFiles:
			treeFile=105 #for debug only
			self.G.terrain.treeFile=treeFile
			self.G.terrain.readTrees()
			for head in ['BC','convAcc']:
				head='BC' #for debug only issues with not ending simulation- interaction after too filled or break
				for nCranes in [1,2]:
					for bundler in [False, True]:
						for twigCrack in [False, True]:
							for simNumber in range(1,it+1):
								G=copy.deepcopy(self.G)
								self.s=ThinningSim(G=G, vis=False, anim=False, head=head, nCranes=nCranes, bundler=bundler, twigCrack=twigCrack)
								self.s.stats['machineConfig']=self.getConfig(head,nCranes,twigCrack,bundler)
								self.s.stats['simNumber']=simNumber
								self.s.stats['treeFile']=treeFile
								self.s.stats['noHarvTrees']=sum([len(b.trees) for b in G.terrain.piles])

								self.s.stats['noCraneCycles']=sum([pb.craneCycles for pb in G.terrain.piles])
								self.s.stats['harvBiomass']=sum([b.weight for b in self.s.m.trees]) #total mass of the trees that were harvested, before tc?
								self.s.stats['harvStemMass']=sum([t.logWeight for t in self.s.m.trees])# total weight of the stems of the trees that are harvested. Not what's in the bundles or piles, but rather what was in terrain before chop.
								self.s.stats['harvStemVol']=sum([t.vol for t in self.s.m.trees])# total volume of the stems of the trees that are harvested. Not what's in the bundles or piles
								self.s.stats['noBundlesOrPiles']=len(G.terrain.piles)
								self.s.stats['minBunPileMass']=min([b.biomass for b in G.terrain.piles])
								self.s.stats['maxBunPileMass']=max([b.biomass for b in G.terrain.piles])
								self.s.stats['totBunPileMass']=sum([b.biomass for b in G.terrain.piles])
								self.s.stats['minBunPileVol']=min([self.getVol(b) for b in G.terrain.piles])
								self.s.stats['maxBunPileVol']=max([self.getVol(b) for b in G.terrain.piles])
								self.s.stats['totBunPileVol']=sum([self.getVol(b) for b in G.terrain.piles])
								self.s.stats['noMainStops']=len(self.s.m.positions) #Here I assume what is meant is number of stops on the mainroad for harvesting. Not number of places with piles close by. Is this a good assumption. Gives seven all the time.. should be len(self.s.m.positions)-1 maybe?
								self.s.stats['totTimeConsumed']=self.s.now()
								if bundler==True:
									self.s.stats['bundlingTime']=self.s.stats['noBundlesOrPiles']*self.s.m.bundler.timeBundle
								else: self.s.stats['bundlingTime']=0
								self.s.stats['work time']#operator active time
								#self.s.stats['totCraneWaitTime']
								#self.s.stats['oneCraneWaitTime']
								#self.s.stats['twoCranesWaitTime']
								#self.s.stats['noCraneWaitings']=
								print self.s.stats['noBundlesOrPiles'], 'piles or bundles'
								print self.s.stats['noCraneCycles'], 'was the number of crane cycles'
								print self.s.stats['bundlingTime'], self.s.stats['totTimeConsumed']
								print '---------------------------------------------------------------------'



								
		#print 'Congratulations, all your simulations has been run and the data has successfully been stored in the excel-file named ********_dateoftoday.xls'

	def getConfig(self,head,nCranes,twigCrack,bundler):
		if nCranes==1: a='A'
		else: a='B'

		if head=='BC' and twigCrack==False: b='c'
		elif head=='BC' and twigCrack==True: b='d'
		elif head=='convAcc' and twigCrack==False: b='e'
		elif head=='convAcc' and twigCrack==True: b='f'
		
		if bundler==False: c='i'
		elif bundler==True: c='j'
		return a+b+c

	def getVol(self,pile):
		if pile.xSection:
			pile.vol=pile.xSection*pile.length
		else:
			pile.vol=pile.length*pi*(pile.diameter/2.)**2
		return pile.vol


class varyAutomationThinningMachine(SimSeries):
	"""
	this class describes a simulation that varies the automation level.
	number of iterations (per stand type and machine type) is the only input variables
	"""
	def __init__(self, it=1, G=None):
		SimSeries.__init__(self,G)
		if not G:
			self.G.areaPoly=[(0,0), (25,0), (25,40), (0,40)]
			self.G.terrain=Terrain(G=self.G)
		fileheader="i\tcranes\t\tcranehead\t\tProd\tQueuePerc\tWorkPerc\tChopPerc\tgroundAratio\tAmove\tAmoveArmin\tAmoveArmOut\tAdumptrees\tAchop"
		a1={'move': False, 'moveArmIn': False, 'moveArmOut': False, 'dumpTrees': False, 'switchFocus': False, 'chop':False}
		a2={'move': False, 'moveArmIn': True, 'moveArmOut': False, 'dumpTrees': False, 'switchFocus': False, 'chop':False}
		a3={'move': False, 'moveArmIn': True, 'moveArmOut': False, 'dumpTrees': True, 'switchFocus': False, 'chop':False}
	   	a4={'move': False, 'moveArmIn': True, 'moveArmOut': True, 'dumpTrees': True, 'switchFocus': False, 'chop':False}
		a5={'move': False, 'moveArmIn': True, 'moveArmOut': True, 'dumpTrees': True, 'switchFocus': False, 'chop':True}
		outpfolder='outputFiles/'+self.__class__.__name__+join([strip(a) for a in split(time.ctime(time.time()))], "_")
		summary={'1a':{}, '2a':{}}			
		for head in ['BC', 'conv']: #just take the regular 1-arm cases first..
			p=np.array([])
			q=np.array([])
			w=np.array([])
			sims=np.array([])
			outT=np.array([])
			stds=[]
			mean=0
			std=0
			meanStd=10000
			for tFile in self.G.terrain.thinningFiles:
				self.outpfolder=outpfolder+'/'+str(tFile)+'/'
				self.filename=self.outpfolder+'/'+'1a'+head+".txt"
				for i in range(it):
					G=copy.deepcopy(self.G)
					G.automatic=a1
					G.terrain.treeFile=tFile
					G.terrain.readTrees()
					s=self._singleSim(G,nCranes=1, head=head)
					if i==0: #first
 						generalInfo=s.getGeneralStatsString() #general statistics
						self._saveToFile(generalInfo)						
						self._saveToFile(fileheader)
						self._saveToFile("####") #indicates that the data starts here.
					sims=np.append(sims,s)
					p=np.append(p, s.stats['productivity'])
					w=np.append(w, s.stats['work percent'])
					q=np.append(q, s.queuePerc)
					outT=np.append(outT, s.stats['outTake'])
					a=a1 #just to have some values for the automatic tasks...
					self._saveToFile(str(i+1)+'\t1\t'+head+
							 '\t%.4f'%s.stats['productivity']+
							 '\t'+'%0.4f'%s.queuePerc+
							 '\t'+'%0.4f'%s.stats['work percent']+
							 '\t'+'%0.4f'%s.stats['outTake']+
							 '\t'+str(a['move'])+
							 '\t'+str(a['moveArmIn'])+
							 '\t'+str(a['moveArmOut'])+
							 '\t'+ str(a['dumpTrees'])+
							 '\t'+str(a['chop']))
				self.f.close()
				self.f=None
			summary['1a'][head]={}
			summary['1a'][head]['A']={'sims':sims, 'p':p, 'w':w, 'q':q, 'outT':outT}
		#2a:
		autolist=[a1, a2, a3, a4, a5]
		astr=['A','B','C','D','E']
		for head in ['BC', 'conv']:
			summary['2a'][head]={}
			for index,a in enumerate(autolist):
				autoStr=astr[index]
				sims=np.array([])
				means=[]
				stds=[]
				p=np.array([])
				q=np.array([])
				w=np.array([])
				outT=np.array([])
				mean=0
				meanStd=10000
				for tFile in self.G.terrain.thinningFiles:
					self.outpfolder=outpfolder+'/'+str(tFile)+'/'
					self.filename=self.outpfolder+'2a'+head+"_"+autoStr+".txt"
					for i in range(it):
						G=copy.deepcopy(self.G)
						G.automatic=a
						G.terrain.treeFile=tFile
						G.terrain.readTrees()
						s=self._singleSim(G,nCranes=2, head=head) #G is copied later, so does not affect G
						if i==0: #first
							generalInfo=s.getGeneralStatsString() #general statistics
							self._saveToFile(generalInfo)						
							self._saveToFile(fileheader)
							self._saveToFile("####") #indicates that the data starts here.
						sims=np.append(sims, s)
 						p=np.append(p, s.stats['productivity'])
						q=np.append(q, s.queuePerc)
						w=np.append(w, s.stats['work percent'])
						outT=np.append(outT, s.stats['outTake'])
						self._saveToFile(str(i+1)+'\t1\t'+head+
							 '\t'+'%.4f'%s.stats['productivity']+
							 '\t'+'%0.4f'%s.queuePerc+
							 '\t'+'%0.4f'%s.stats['work percent']+
							 '\t'+'%0.4f'%s.stats['outTake']+
							 '\t'+'%0.4f'%s.stats['groundAreaRatio']+
							 '\t'+str(a['move'])+
							 '\t'+str(a['moveArmIn'])+
							 '\t'+str(a['moveArmOut'])+
							 '\t'+str(a['dumpTrees'])+
							 '\t'+str(a['chop']))
					self.f.close()
					self.f=None
					print "No of sims: %d"%(len(sims))
				summary['2a'][head][autoStr]={}
				summary['2a'][head][autoStr]['sims']=sims
				summary['2a'][head][autoStr]['p']=p
				summary['2a'][head][autoStr]['w']=w
				summary['2a'][head][autoStr]['q']=q
				summary['2a'][head][autoStr]['outT']=outT
		#print the summary.
		self.outpfolder=outpfolder
		self.filename=outpfolder+'/'+'summary.txt'
		fileheader="iter\tcranes\thead\tmeanprod\tstdOfmeanProd\tmeanQueue\tstdMeanQueue\tmeanWorkPerc.\tstdMeanW\tmeanOutT\tstdMeanOutT\tAmove\tAmoveArmin\tAmoveArmOut\tAdumptrees\tAchop"
		self._saveToFile(fileheader)
		self._saveToFile("####") #indicates that the data starts here.
		for head in ['BC', 'conv']:
			for arms in [1, 2]:
				if arms==1: aList=['A']
				else: aList=astr
				for a in aList:
					d=summary[str(arms)+'a'][head][a] #the saved data (in a dictionary)..
					auto=d['sims'][0].m.automatic
					self._saveToFile(
						str(i+1)+'\t'+
						str(arms)+'\t'+
						head+'\t'+
						'%0.4f'%np.mean(d['p'])+'\t'+
						'%0.4f'%(np.std(d['p'])/sqrt(len(d['p'])))+'\t'+
						'%0.4f'%np.mean(d['q'])+'\t'+
						'%0.4f'%(np.std(d['q'])/sqrt(len(d['q'])))+'\t'+
						'%0.4f'%np.mean(d['w'])+'\t'+
						'%0.4f'%(np.std(d['w'])/sqrt(len(d['w'])))+'\t'+
						'%0.4f'%np.mean(d['outT'])+'\t'+
						'%0.4f'%(np.std(d['outT'])/sqrt(len(d['outT'])))+'\t'+
					   	str(auto['move'])+'\t'+
						str(auto['moveArmIn'])+'\t'+
						str(auto['moveArmOut'])+'\t'+
						str(auto['dumpTrees'])+'\t'+
						str(auto['chop']))
		self.f.close()

	def _singleSim(self, G, nCranes=2, head='BC'):
		"""
		a single run.
		"""
		SimExtend.seed='undefined'
		s=ThinningSim(G,vis=False,nCranes=nCranes, head=head)
		return s

def createThinningMovies():
	G=globalVar()
	G.areaPoly=[(0,0), (25,0), (25,40), (0,40)]
	G.terrain=Terrain()
	G.terrain.readTrees('terrainFiles/thinning/GA-210.DAT') #the best treefile
	ThinningSim(G=G,vis=False, anim=True, head='BC', nCranes=2)
	try:
		os.rename('animation.mpg', 'BC2arm.mpg')
	except: pass
	ThinningSim(G=G,vis=False, anim=True, head='BC', nCranes=1)
	try:
		os.rename('animation.mpg', 'BC1arm.mpg')
	except: pass
	ThinningSim(G=G,vis=False, anim=True, head='conv', nCranes=1)
	try:
		os.rename('animation.mpg', 'conventionalHead.mpg')
	except: pass

class corridorStatistics(SimSeries):
	def __init__(self,i=10):
		SimSeries.__init__(self,G=None)
		if not self.G.areaPoly:
			G.areaPoly=[(0,0), (25,0), (25,40), (0,40)]
		self.G.terrain=Terrain(G=self.G)
		fileheader="i\tcranehead\tprod\ttreefile\tnCorr\tlogsPerCorr\tlogsInMainRoad\tCorrCumLogW\t"
		self._saveToFile(fileheader)
		self._saveToFile("####")
		j=0
		for head in ['BC', 'conv']:
			for tFile in self.G.terrain.thinningFiles:
				for it in range(i):
					G=copy.deepcopy(self.G)
					G.terrain.treeFile=tFile
					G.terrain.readTrees()
					s=self._singleSim(G,nCranes=2, head=head)
					p=s.stats['productivity']
					self._saveToFile("%d\t"%j+head+'\t'+'%.6f\t'%p+str(tFile)+'\t'+str(s.cNum)+'\t'+'%.6f'%s.avTreesPerCorr+'\t'+str(s.treesPerRoad)+'\t'+'%.6f'%s.avCorrTreeWeight)
					j+=1
		self.f.close()

	def _singleSim(self, G, nCranes=2, head='BC'):
		"""a single run."""
		SimExtend.seed='undefined'
		s=ThinningSim(G,vis=False,nCranes=nCranes, head=head)
		return s

def setDefaultThinningParams(simParam={}):
	"""
	sets default values for all the relevant keys.
	A means: A machine with one crane
	B means: A machine with two cranes
	C means: Continuous head (Rickard och Julia)
	D means: Continuous head with twig cracking and logging
	E means: Conventional head which can accumulate trees
	F means: Conventional head which can accumulate trees and twig crack and log
	J means: Bundler module in front of the machine which can make bundles of the trees and log them
	"""
	s=simParam
	
	#OVERALL
	s['maxCraneLength']=11 #[m]
	s['minCraneLength']=3 #[m]
	s['corridorWidthEF']=1 #[m]
	s['corridorWidthCD']=2 #[m]
	s['noCorridorsPerSideCD']=5 #Number of corridors per side
	s['noCorridorsPerSideEF']=3 #Number of corridors per side
	s['maxCorridorAngle']=45#[deg] angle of the corridor in relation to the strip road
	s['switchFocusTime']=3#Time to switch focus from one thing to another
	s['noIter']=100#number of iterations per configuration

	#MACHINES A,B
	s['moveConst']=5#[s]
	s['velocityOfMachine']=1#[m/s]
	s['radialVelocityOfCrane']=2.5#[m/s]
	s['moveCraneConst']=1.5#[s]
	s['angularVelocityOfCrane']=0.35#[rad/s]
	s['maxPower']=150 #[kW] Maximum powerthe machine can operate at
	s['minAngleForward']=15#[degrees] Minimum angle for cranes to striproad without machine tipping... Necessary?
	s['powToMove']=100 #[kW]The power it takes to have the machine moving
	
	#AUTOMATION
	s['moveMachine']=False#
	s['chooseCorridor']=False#
	s['moveArmOutCD']=False#
	s['fellTreesCD']=True#
	s['moveArmInCD']=True#
	s['dropTreesCD']=False#
	s['cuttingD']=False#
	s['twigCrackD']=False#
	s['moveArmOutEF']=False#
	s['fellTreesEF']=False#
	s['moveArmInEF']=False#
	s['dropTreesEF']=False#
	s['cuttingF']=False#
	s['twigCrackF']=False#
	s['restOfBundling']=True# Should the rest of the bundling process be automatic?
	s['startBundler']=False# Need the dirver push a button or is it made automatically?
	
	#HEADS C,D,E,F
	s['headWidthCD']=1
	s['headWidthEF']=0.5
	s['velocityFellTreeCD']=0.08 #[m2/s] Velocity of the cutting
	s['velocityFellTreeEF']=0.08 #[m2/s] Velocity of the cutting
	s['constFellTreeCD']=0
	s['constFellTreeEF']=1
	s['timeDropTreesCD']=10#[s] Time it takes to drop the trees for the continuous head
	s['timeDropTreesEF']=10#[s] Time it takes to drop the trees for the conventional head
	s['timeTwigCrack']=5#[s] Time it takes to twig crack a bunch of trees
	s['timeCut']=5#[s] Time it takes to log the treas at the head
	s['maxWeightCD']=350#[kg] Maximum weight load.
	s['maxWeightEF']=350#[kg] Maximum weight load.
	s['maxGripAreaCD']=0.3#[m2] Maximum grip area for the head. This controls how much it can accumulate C and D
	s['maxGripAreaEF']=0.3#[m2]-----------"---------------------------- E and F
	s['powMoveCraneCD']=20#[kW] The power it takes to move the crane, N.B. larger than for EF due to moving cutting blade
	s['powMoveCraneEF']=15#[kW] The power it takes to move the crane
	s['powHoldLoadCD']=1#[kW/kg] The power it takes for the crane to hold a load of 1 kg
	s['powHoldLoadAtDistCD']=1#[kW/m] the power it takes for the crane to hold a load per meter from machine
	s['powHoldLoadEF']=1#
	s['powHoldLoadAtDistEF']=1#
	s['powTwigCrack']=5#[kW] power demanded to twig crack the trees at the head
	s['powCut']=1# [kW] power demanded to cut trees at the head

	#BUNDLER J
	s['dropPosJ']=3#[m] At what position the cranes should drop the trees. Given is distance in front of crane center
	s['timeBundle']=60#[s] Time it takes to finish bundling of trees in the bundler, when no twig cracking at heads
	s['timeStartBundler']=1#[s] The time it takes to get the bundler going. (Push the button)
	s['powBundleJ']=1#[kW] Power it takes to make a bundle at the bundler J
	s['maxXSectionJ']=0.5#[m2] Maximum cross section of "stored" trees in bundler
	s['xSectionThreshJ']=0.4#[m2] Cross section for which the bundler makes a bundle without waiting for a new load from a head.
	#PRIORITIES
	#--


	

###############################
# Thinning sim
###############################
class ThinningSim(SimExtend):
	"""
	class for a single simulation with a 1a or 2a thinning machine
	"""
	def __init__(self, G=None, vis=True, anim=False, head='BC',nCranes=2,series=None, bundler=False, twigCrack=False):
		SimExtend.__init__(self,G, vis, anim, animDelay=1.2,series=series) #does some common stuff, e.g. seed

		#if no file to read the simulationsparameters from:
		setDefaultThinningParams(self.G.simParam) #sets the parameters to default values
			
		if not self.G.terrain:
			self.G.areaPoly=[(0,0), (25,0), (25,40), (0,40)] #default for thinning files.
			self.G.terrain=Terrain(G=self.G)
			self.G.terrain.readTrees(thinning=True)
		craneMax=self.G.simParam['maxCraneLength']
		startPos=[random.uniform(craneMax, 25-craneMax), -4]
		self.m=ThinningMachine(name="thinny", sim=self, G=self.G, head=head, nCranes=nCranes,startPos=startPos, bundler=bundler,twigCrack=twigCrack)
		self.treeStats() #do some statistics on the trees
		self.activate(self.m,self.m.run())
		self.simulate(until=10000)
		#some simulation information:
		self.stats['productivity']=len(self.m.trees)/self.now()*3600
		self.stats['outTake']=100*len(self.m.trees)/float(len(self.G.terrain.trees))
		self.stats['groundAreaRatio']=sum([t.dbh**2*pi*0.25 for t in self.m.trees])/max(1,sum([t.dbh**2*pi*0.25 for t in self.G.terrain.trees]))
		
		print "Simulation ended. Productivity: %.1f trees/hour. %.1f %% of the trees were harvested"%(len(self.m.trees)/self.now()*3600, self.stats['outTake'])
		print "average trees per corridor: %f average log weight in corridor: %f trees in main road: %f"%(self.avTreesPerCorr, self.avCorrTreeWeight, self.treesPerRoad)
		self.setQueuePerc()
		print self.getGeneralStatsString()
		if self.p: #plot
			self.p.terminate()
			if vis: plt.show()

	def treeStats(self):
		"""
		calculates some statistics.. should maybe be done in terrain class instead?
		"""
		trees=float(len(self.G.terrain.trees))
		self.saveGeneralStats("stand no", self.G.terrain.treeFile, "-") #see simSeries for how it works..
		self.saveGeneralStats("trees", trees, "-")
		if trees==0: return None
		self.saveGeneralStats("pine percentage", 100*len([t for t in self.G.terrain.trees if t.specie=='pine'])/trees, "%")
		self.saveGeneralStats("spruce percentage", 100*len([t for t in self.G.terrain.trees if t.specie=='spruce'])/trees, "%")
		self.saveGeneralStats("leaf tree percentage", 100*len([t for t in self.G.terrain.trees if t.specie=='leaf'])/trees, "%")
		treeDensity=trees/self.G.terrain.area
		self.saveGeneralStats("tree density", treeDensity, "trees/m2")
		mLogs=len(self.m.mainRoadTrees)
		mNum=0
		cLogs=len(self.m.corridorTrees)
		cNum=0
		for r in self.m.roadList:
			if r.radius>2*self.m.craneMaxL/2.+2: #main road.
				mNum+=1
			else:
				cNum+=1
		self.cNum=cNum
		self.mNum=mNum
		self.mLogs=mLogs
		self.cLogs=cLogs
		cWeight=sum([t.logWeight for w in self.m.corridorTrees ])
		mWeight=sum([t.logWeight for w in self.m.corridorTrees ])
		print "number of main roads:", mNum, "number of corridors:", cNum
		if cNum==0: self.avTreesPerCorr=0
		else: self.avTreesPerCorr=cLogs/float(cNum)
		self.treesPerRoad=mLogs
		if cLogs==0: self.avCorrTreeWeight=0
		else: self.avCorrTreeWeight=cWeight/float(cNum)
		if mLogs==0: self.avRoadTreeWeight=0
		else: self.avRoadTreeWeight=mWeight/float(mNum) #the cumulative sum
		#clustering...  see Clark and Evans 1954
		rexp=0.5*1/(sqrt(treeDensity)) #expected r
		robs=0
		for t in self.G.terrain.trees:
			tmp=[]
			R=5
			while len(tmp)==0 and R<sqrt(self.G.terrain.area): #in 99.9% of the cases, this loop only runs once.
				#we are not using self.G.terrain.trees for speedup reasons.
				tmp=self.G.terrain.GetTrees(t.pos, R)
				R+=2
			if len(tmp)==0:
				self.saveGeneralStats("clustering agg. index", "no trees", "trees/m2")
				break
			shortest=1e10
			for treeTmp in tmp:
				if treeTmp==t: continue #same tree
				d=fun.getDistance(t.pos, treeTmp.pos)
				if d<shortest: shortest=d
			if shortest==1e10: raise Exception('could not find shortest tree...')
			robs+=shortest
		robs/=trees
		print "rexp:", rexp, "robs.", robs
		aggregationIndex=robs/rexp
		self.saveGeneralStats("clustering agg. index", aggregationIndex, "-")

	def plotMoni(self, ax, number):
		tcap=50
		tmin=-0.1
		if self.now()<tcap: t=tcap
		else:
			if self.now()-tmin>400: tmin=self.now()-400
			t=self.now()
		if number==1: #plot driver activity
			driver=self.m.driver
			moni=driver.actMon
			if len(moni)>0:
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([tmin, t, -0.1, 1.2])
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
				ax.axis([tmin, t, -0.1, max(moni.yseries())+0.1])
				ax.grid(True)
				ax.set_ylabel('Wait queue for driver.')
		elif number==3: #plot trees planted
			moni=self.m.treeMoni
			#trees planted
			if len(moni)>0:
				ax.plot(moni.tseries(),moni.yseries(), drawstyle='steps-post')
				ax.axis([-0.1, t, -0.1, (max(moni.yseries())+1)])
				ax.grid(True)
				ax.set_xlabel('time(s)')
				ax.set_ylabel('trees harvested')
		return ax
