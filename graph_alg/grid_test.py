from mpl_toolkits.mplot3d import axes3d, Axes3D # <-- NOTE!
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from matplotlib import cm

#open

def readTerrain(name=None):
	"""
	reads an .asc terrain file and returns grid coordinates.
	"""
	if not name: name='tab/67275_5950_25.asc'
	f = open(name, 'r')

	xrange=[1500, 1800]
	yrange=[1300, 1600]
	if xrange[0]>xrange[1] or yrange[0]>yrange[1]: raise Exception('invalid ranges.')
	x, y, z=[], [], []
	xlist, ylist, zlist=[], [], []
	m=1 #modulu, to reduce the grid resolution
	yold=yrange[0]
	for index, line in enumerate(f):
		l=line.split()
		if len(l)==0: break
		if index==0:
			xin=int(l[0])
			yin=int(l[1])
		xtmp=int(l[0])-xin
		ytmp=int(l[1])-yin
		if ytmp>yrange[1]: break
		if xtmp<=xrange[1] and xtmp>=xrange[0] and ytmp>=yrange[0] and xtmp%m==0 and ytmp%m==0:
			ztmp=float(l[2])
			if ytmp != yold: #new row..
				x.append(xlist)
				y.append(ylist)
				z.append(zlist)
				xlist, ylist, zlist = [], [], []
			xlist.append(xtmp)
			ylist.append(ytmp)
			zlist.append(ztmp)
			yold=ytmp
	f.close()
	return x,y,z
def plotSurface(x,y,z):
	"""
	plots the surface of a terrain grid given x,y,z
	"""
	if not z or not y or not x: raise Exception('x,y,z needs to be given.')
	if len(z)!=len(y) or len(y)!=len(x): raise Exception('lengths of vectors needs to be identical')
	elmax=200000000 #will not plot more than this number of elements
	fig = plt.figure()
	ax = Axes3D(fig)
	ax.grid(True)
	x,y,z=np.array(x),np.array(y),np.array(z)
	el=len(x[0])*len(y)
	step=np.ceil(float(el)/elmax)
	print "step:", step, " elements:", el, "elements used:", el/step**2
	ax.plot_surface(x,y,z, rstride=step, cstride=step, cmap=cm.autumn)
	ax.set_zlim3d(50, 105)
	ax.set_title('terrain')
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_zlabel('z [m]')
	
	fig = plt.figure()
	ax2 = Axes3D(fig)
	ax2.grid(True)
	ax2.contour(x,y,z, zdir='z')
	ax2.set_zlim3d(50, 105)
	return ax
if __name__=='__main__':
	coord=readTerrain()
	ax=plotSurface(*coord)
	plt.show()
