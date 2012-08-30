import numpy as np
import scipy.interpolate as si
import scipy.spatial.distance as ssd

curve0 = np.loadtxt("curve0.txt")
curve1 = np.loadtxt("curve1.txt")


Fx = si.Rbf(curve0[:,0], curve0[:,1],curve1[:,0],function="thin-plate")
Fy = si.Rbf(curve0[:,0], curve0[:,1],curve1[:,1],function="thin-plate")

Xs0,Ys0 = np.mgrid[0:101:5, 0:101:5]
Xs0, Ys0 = Xs0.flatten(), Ys0.flatten()

Xs1 = Fx(Xs0, Ys0)
Ys1 = Fy(Xs0, Ys0)



from pylab import *
plot(curve0[:,0],curve0[:,1],'k')
plot(curve1[:,0],curve1[:,1],'k')
plot(Xs1.flatten(),Ys1.flatten(),'r.')