#!/usr/bin/env python
import sim
import sim.thinning.thinningSim as TH
import sim.planting.PM as PM
import sim.forwarder.forwSim as FW
import sim.tools as tools
import functions as fun
from math import *
import random
import copy
import time
import os, sys, shutil
import numpy as np
##########################################################
## MAIN FUNCTION
##########################################################
if __name__=='__main__':
	anim=False
	iterations=2 #default
	if len(sys.argv)>1 and sys.argv[1] == 'anim':
		anim=True
	elif len(sys.argv)>2:
		a=eval(sys.argv[2])
		iterations=a
		if len(sys.argv)>3:#this should be only for the bashscript
			head=str(sys.argv[3])
			nCranes=eval(sys.argv[4])
			bundler=eval(sys.argv[5])
			twigCrack=eval(sys.argv[6])
			simNumber=eval(sys.argv[7])
			rowNumber=eval(sys.argv[8])
			treeFile=eval(sys.argv[9])
		
	if not anim and len(sys.argv)>1:
		if sys.argv[1] == 'findBugs':
			sim.tools.findBugs()
		elif sys.argv[1] == 'tryItAll': #tests for all simulations so far. See if they throw exception
			for i in range(4):
				s=PM.PlantmSim(vis=False,anim=anim,mtype=random.choice(['1a1h', '1a2h', '2a2h', '2a4h']))
				assert s.productivity>10
				s=TH.ThinningSim(vis=False,anim=anim, head=random.choice(['convAcc', 'BC']), nCranes=random.choice([1,2]), bundler=random.choice([True, False]), twigCrack=random.choice([True, False]))
				assert s.stats['productivity']>10
				FW.ForwarderSim(vis=False, anim=anim)
			print "--------------"
			print "you passed the test. Feel free to commit."
		elif sys.argv[1]=='varyCrane':
			rMax=np.linspace(9,12,15)
			rMin=4
			rList=[]
			for R in rMax:
				rList.append((rMin,R))
			for ttype in [1,2,'2b',3,4,5,6,7]:
				PM.varyCraneRadius(i=20, mtype='2a2h', ttype=str(ttype), G=None, rList=rList)		
		elif sys.argv[1]=='VaryCraner':
			PM.VaryCraneRadiusPMachine()
		elif sys.argv[1]=='VaryTerrain':
			PM.VaryTerrain(iterations)
		elif sys.argv[1]=='VaryAutoPM':
			for ttype in [1,2,'2b',3,4,5,6,7]:
				PM.VaryAutomationPMachine(i=40,ttype=str(ttype), G=None)
		elif sys.argv[1]=='VaryAutoTh':
			TH.varyAutomationThinningMachine(it=iterations)
		elif sys.argv[1]=='thinningMovies':
			TH.createThinningMovies()
		elif sys.argv[1]=='VaryBladeW':
			for ttype in [1,2,'2b',3,4,5,6,7]:
				PM.VaryBladeWidthPMachine(i=15,ttype=ttype)
		elif sys.argv[1]=='VaryAng':
			th1List=np.linspace(-15, 15, 16)
			th2=45
			aList=[]
			for th1 in th1List:
				aList.append([th1*pi/180., th2*pi/180.])
			PM.varyAnglesPMachine(ttype=3, i=30, aList=aList)
			th2List=np.linspace(30, 60, 16)
			th1=0
			aList=[]
			for th2 in th2List:
				aList.append([th1*pi/180., th2*pi/180.])
			varyAnglesPMachine(ttype=3, i=30, aList=aList)
		elif sys.argv[1]=='corrStat':
			TH.corridorStatistics(i=100)
		elif sys.argv[1]=='VaryAttachLoc':
			PM.VaryAttachLoc(iterations)
		elif sys.argv[1]=='VaryDegrees':
			PM.VaryDegrees(iterations)
		elif sys.argv[1]=='VaryDibbleDist':
			PM.VaryDibbleDist(iterations)
		elif sys.argv[1]=='VaryMoundingbladeWidth':
			PM.VaryMoundingbladeWidth(iterations)
		elif sys.argv[1]=='VaryTargetStockingrate':
			PM.VaryTargetStockingrate(iterations)
		elif sys.argv[1]=='VaryMinDist':
			PM.VaryMinDist(iterations)
		elif sys.argv[1]=='VaryOperatorSwitchFocusSkill':
			PM.VaryOperatorSwitchFocusSkill(iterations)
		elif sys.argv[1]=='VaryAutomation':
			PM.VaryAutomation(iterations)
		elif sys.argv[1]=='VaryRadialCraneSpeed':
			PM.VaryRadialCraneSpeed(iterations)
		elif sys.argv[1]=='BestCase':
			PM.BestCase(iterations)
		elif sys.argv[1]=='WorstCase':
			PM.WorstCase(iterations)
		elif sys.argv[1]=='varyAll':
			PM.varyAll(iterations)
		elif sys.argv[1]=='tDCTM':#tryDiffConfigThinningMachine
			TH.tryDiffConfigThinningMachine(it=100)
		elif sys.argv[1]=='btDCTM':#bashtryDiffConfigThinningMachine
			TH.bashTryDiffConfigThinningMachine(it=iterations, head=head, nCranes=nCranes, bundler=bundler, twigCrack=twigCrack, simNumber=simNumber, rowNumber=rowNumber, treeFile=treeFile)
		else:
			raise Exception('could not read input argument %s'%str(sys.argv[1]))
		
	else:
		import cProfile
		from sim.planting.plantMTerrain import PlantMTerrain
		seed=int(random.uniform(0,100000))
		#seed=35342
		random.seed(seed)
		print "seed2:", seed
		G=tools.globalVar()
		G.noMonitors=True
		G.seed=seed
		#TH.testMemory()
		#G.terrain=PlantMTerrain(G, ttype='0')
	   	#s=PM.PlantmSim(vis=True,anim=anim,G=G, mtype='2a4h')
		#TH.ThinningSim(vis=True,anim=anim,G=G, head='BC', nCranes=2)
		#G.terrain=PlantMTerrain(G, ttype='5')
		G.plotDelay=10
	   	#s=PM.PlantmSim(vis=True,anim=anim,G=G, mtype='2a4h')
		TH.ThinningSim(vis=True,anim=anim,G=G, head='BC', nCranes=2, bundler=False, twigCrack=False, observer=False)
		#FW.ForwarderSim(vis=True, anim=anim, G=G)
		#FW.simRandomRoad(vis=True, anim=anim, G=G)
		#cProfile.run('FW.ForwarderSim(vis=True, anim=anim, G=G)')
		#cProfile.run('''PM.PlantmSim(vis=False,G=G,anim=anim, mtype='2a2h', ttype=4)''')
