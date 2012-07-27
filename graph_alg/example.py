"""
this is an example of how it could work.
just a script. Not a part of the "software-code"..
"""
import os, sys #insert /dev to path so we can import these modules.
if __name__=='__main__':
	cmd_folder = os.path.split(os.path.dirname(os.path.abspath(__file__)))[0]
	if not cmd_folder in sys.path:
		sys.path.insert(0, cmd_folder)
import GIS.GIS as GIS
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
import numpy as np
import random
import copy

from draw import draw_custom, draw_road
import graph_operations as go
import costFunctions as cf
import functions as fun
import grid as gr
import spider_grid as sg
from algorithms.bruteForce import bruteForce
from algorithms.simplified_bruteForce import simplified_bruteForce
from algorithms.stochastic import stochastic, stochastic_several, ProbListGen



###############
# This is just a bunch of Linus' scripts.. not a part of the "program/software"
#################


import cProfile
import time
import random
tic=time.clock()
#areaPoly=list(3*np.array([(13,74),(48,0), (-75, 25), (-35,96)]))
areaPoly=[(522, 1048),(541, 1017),(613, 997),(661,955),(681, 901),(709,883),(711,870),(858,708),(874,708),(907,640),(933, 622),(973, 553),(973,529),(885,507),(831, 468),(828, 442),(850, 421),(859,406),(871, 396),(876,375),(868,361),(460, 289),(367, 292),(339, 316),(319,378),(303, 408),(301, 432),(375, 474),(432, 487),(496, 529),(496, 558),(450, 616),(433,640),(376, 693),(366,711),(352, 711),(357, 838),(396, 901),(403, 940),(445, 993),(475, 1012),(487, 1029),]
brazil=[(3,189),(16,207),(23,213),(29,222),(39,218),(45,212),(47,229),(51,221),(53,228),(69,229),(94,214),(111,211),(112,233),(127,245),(139,246),(157,257),(173,259),(177,289),(199,290),(199,303),(209,311),(201,337),(205,361),(230,363),(235, 386),(243,383),(249,386),(245,405),(251,404),(254,423),(212,463),(218,461),(229,474),(233,471),(260, 493),(257,507),(271,485),(283,462),(289,464),(281,478),(289,473),(309,442),(315,430),(313,405),(350,385),(367,375),(390,375),(400,366),(419, 334),(428,315),(435,251),(443,253),(458,228),(487,201),(483,159),(458,153),(427,126),(385,117),(370,123),(370,111),(327,93),(309,115),(309,105),(322,92),(293,89),(287,99),(285,90),(302,75),(303,66),(291,59),(283,35),(268,58),(226,56),(227,63),(210,63),(189,71),(179,64),(181,39),(171,23),(167,22),(167,28),(136,43),(117,36),(125,57),(131,60),(104,81),(87,76),(80,65),(50,70),(51,78),(60,79),(59,84),(49,83),(47,93),(56,106),(50,145),(14,157),(12,174),(5,180)]

areaPoly=brazil
areaPoly=list(0.6*np.array(brazil))
for i in range(len(areaPoly)):
	areaPoly[i]=(areaPoly[i][0], 1050-areaPoly[i][1])
for i in range(len(areaPoly)): areaPoly[i]=tuple(areaPoly[i])
#cProfile.run("tmp(areaPoly)")

globalOrigin= 596250, 6727996 #coordinate
#R=gr.SqGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin, angle=3.14/4+0.1)
R=gr.TriGridGraph(areaPoly=areaPoly, globalOrigin=globalOrigin)
print "algorithm time... "
R=simplified_bruteForce(R,aCap=0.2,warmup=False, anim=False)
#R=bruteForce(R,aCap=0.2, add=False)
#R=best(areaPoly, t=60)
R.draw(overlap=True)

#testInterpolation()
#best(areaPoly,1, origin=(437,226), direction=3.14/2*0.05)
#tryP0(areaPoly, t=60)
#findBugs([simplified_bruteForce, stochastic])

#testAngles()
#compareAddAndDontAdd()
#compareBruteAndSimpleBrute()
print "program took: ", time.clock()-tic, " seconds"
plt.show()
