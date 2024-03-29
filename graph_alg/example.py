"""
this is an example of how it could work.
just a script. Not a part of the "software-code"..
"""
import os, sys #insert /dev to path so we can import these modules.
if __name__=='__main__':
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import matplotlib.pyplot as plt
import numpy as np
import random
import copy
import time

import grid as gr
import spider_grid as sg
from algorithms.bruteForce import bruteForce
from algorithms.simplified_bruteForce import simplified_bruteForce
from algorithms.stochastic import stochastic, stochastic_several, ProbListGen



###############
# This is just an example script of how the program can be used. Look at the bottom.
#################


tic=time.clock()

#a bunch of example polygons
poly0=list(3*np.array([(0,0), (75, 25), (35,96), (-13,74),]))

poly1=[(6.75, -99.75), (36.0, -59.25), (51.75, -45.75), (65.25, -25.5), (87.75, -34.5), (101.25, -48.0), (105.75, -9.75), (114.75, -27.75), (119.25, -12.0), (155.25, -9.75), (211.5, -43.5), (249.75, -50.25), (252.0, -0.75), (285.75, 26.25), (312.75, 28.5), (353.25, 53.25), (389.25, 57.75), (398.25, 125.25), (447.75, 127.5), (447.75, 156.75), (470.25, 174.75), (452.25, 233.25), (461.25, 287.25), (517.5, 291.75), (528.75, 343.5), (546.75, 336.75), (560.25, 343.5), (551.25, 386.25), (564.75, 384.0), (571.5, 426.75), (477.0, 516.75), (490.5, 512.25), (515.25, 541.5), (524.25, 534.75), (585.0, 584.25), (578.25, 615.75), (609.75, 566.25), (636.75, 514.5), (650.25, 519.0), (632.25, 550.5), (650.25, 539.25), (695.25, 469.5), (708.75, 442.5), (704.25, 386.25), (787.5, 341.25), (825.75, 318.75), (877.5, 318.75), (900.0, 298.5), (942.75, 226.5), (963.0, 183.75), (978.75, 39.75), (996.75, 44.25), (1030.5, -12.0), (1095.75, -72.75), (1086.75, -167.25), (1030.5, -180.75), (960.75, -241.5), (866.25, -261.75), (832.5, -248.25), (832.5, -275.25), (735.75, -315.75), (695.25, -266.25), (695.25, -288.75), (724.5, -318.0), (659.25, -324.75), (645.75, -302.25), (641.25, -322.5), (679.5, -356.25), (681.75, -376.5), (654.75, -392.25), (636.75, -446.25), (603.0, -394.5), (508.5, -399.0), (510.75, -383.25), (472.5, -383.25), (425.25, -365.25), (402.75, -381.0), (407.25, -437.25), (384.75, -473.25), (375.75, -475.5), (375.75, -462.0), (306.0, -428.25), (263.25, -444.0), (281.25, -396.75), (294.75, -390.0), (234.0, -342.75), (195.75, -354.0), (180.0, -378.75), (112.5, -367.5), (114.75, -349.5), (135.0, -347.25), (132.75, -336.0), (110.25, -338.25), (105.75, -315.75), (126.0, -286.5), (112.5, -198.75), (31.5, -171.75), (27.0, -133.5), (11.25, -120.0)]

brazil=[(4.5, 766.5), (24.0, 739.5), (34.5, 730.5), (43.5, 717.0), (58.5, 723.0), (67.5, 732.0), (70.5, 706.5), (76.5, 718.5), (79.5, 708.0), (103.5, 706.5), (141.0, 729.0), (166.5, 733.5), (168.0, 700.5), (190.5, 682.5), (208.5, 681.0), (235.5, 664.5), (259.5, 661.5), (265.5, 616.5), (298.5, 615.0), (298.5, 595.5), (313.5, 583.5), (301.5, 544.5), (307.5, 508.5), (345.0, 505.5), (352.5, 471.0), (364.5, 475.5), (373.5, 471.0), (367.5, 442.5), (376.5, 444.0), (381.0, 415.5), (318.0, 355.5), (327.0, 358.5), (343.5, 339.0), (349.5, 343.5), (390.0, 310.5), (385.5, 289.5), (406.5, 322.5), (424.5, 357.0), (433.5, 354.0), (421.5, 333.0), (433.5, 340.5), (463.5, 387.0), (472.5, 405.0), (469.5, 442.5), (525.0, 472.5), (550.5, 487.5), (585.0, 487.5), (600.0, 501.0), (628.5, 549.0), (642.0, 577.5), (652.5, 673.5), (664.5, 670.5), (687.0, 708.0), (730.5, 748.5), (724.5, 811.5), (687.0, 820.5), (640.5, 861.0), (577.5, 874.5), (555.0, 865.5), (555.0, 883.5), (490.5, 910.5), (463.5, 877.5), (463.5, 892.5), (483.0, 912.0), (439.5, 916.5), (430.5, 901.5), (427.5, 915.0), (453.0, 937.5), (454.5, 951.0), (436.5, 961.5), (424.5, 997.5), (402.0, 963.0), (339.0, 966.0), (340.5, 955.5), (315.0, 955.5), (283.5, 943.5), (268.5, 954.0), (271.5, 991.5), (256.5, 1015.5), (250.5, 1017.0), (250.5, 1008.0), (204.0, 985.5), (175.5, 996.0), (187.5, 964.5), (196.5, 960.0), (156.0, 928.5), (130.5, 936.0), (120.0, 952.5), (75.0, 945.0), (76.5, 933.0), (90.0, 931.5), (88.5, 924.0), (73.5, 925.5), (70.5, 910.5), (84.0, 891.0), (75.0, 832.5), (21.0, 814.5), (18.0, 789.0), (7.5, 780.0)]

#choose/create a polygon. 
areaPoly=poly0
for i in range(len(areaPoly)): #just some type conversion
	areaPoly[i]=tuple(areaPoly[i])

#give a coordinate. Could be skipped, then default is used
globalOrigin= 596250, 6727996 #SWEREF99 coordinate around sandviken in this case

#create a grid from our polygon
from weightFunctions import normPitchRollDist
R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin, weightFunction=normPitchRollDist)
#R=gr.TriGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
#R=sg.SpiderGridGraph(origin=(0,0),areaPoly=areaPoly, globalOrigin=globalOrigin) #a little buggy. beware..


#choose an algorithm to modify the grid. More arguments can be given, look at the code..
#to choose a different algorithm, just uncomment it.
R=simplified_bruteForce(R,aCap=0.2)
#R=stochastic(R, aCap=0.2)
#R, tries=stochastic_several(R, aCap=0.2, t=60*15) #time available in seconds to get best solution

print "program took: ", time.clock()-tic, " seconds"

#we are done. Now plot it.
R.draw()
plt.show()
