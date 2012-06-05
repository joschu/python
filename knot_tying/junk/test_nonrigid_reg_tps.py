import numpy as np
import scipy.interpolate as si
import scipy.spatial.distance as ssd

curve0 = np.loadtxt("curve0.txt")
curve1 = np.loadtxt("curve1.txt")

class ThinPlateSpline:
    def __init__(self, xs, ys, ds):
        dists = ssd.squareform(ssd.pdist(np.c_[xs,ys]))
        
        self.xs, self.ys = xs, ys
        
        K = dists**2 * np.log(dists)
        K[range(K.shape[0]), range(K.shape[1])]=0        
        P = np.c_[np.ones(xs.size), xs, ys]
        
        L = np.r_[
             np.c_[K,   P],
             np.c_[P.T, np.zeros((3,3))]]

        Y = np.r_[ds, 0,0,0]
        waaa = np.dot(np.linalg.inv(L), Y)
        self.w, self.a0, self.ax, self.ay = waaa[:-3], waaa[-3], waaa[-2], waaa[-1]
        
    def __call__(self, xis, yis):
        dists = ssd.cdist(np.c_[xis, yis], np.c_[self.xs, self.ys])
        ks = dists**2 * np.log(dists)
        ks[np.isnan(ks)] = 0
        return self.a0 + xis*self.ax + yis*self.ay + np.dot(ks,self.w)

    def grad(self, xis, yis):
        dists = ssd.cdist(np.c_[xis, yis], np.c_[self.xs, self.ys])
        
        xdiffs = xis [:,None] - self.xs[None,:]
        ydiffs = yis[:,None] - self.ys[None,:]
        xgrad = self.ax + np.dot((2*np.log(dists) + 1) * xdiffs, self.w)
        ygrad = self.ay + np.dot((2*np.log(dists) + 1) * ydiffs, self.w)
                
        return np.c_[xgrad, ygrad]
        


Fx = ThinPlateSpline(curve0[:,0], curve0[:,1],curve1[:,0])
Fy = ThinPlateSpline(curve0[:,0], curve0[:,1],curve1[:,1])

Xs0,Ys0 = np.mgrid[0:101:5, 0:101:5]
Xs0, Ys0 = Xs0.flatten(), Ys0.flatten()

Xs1 = Fx(Xs0, Ys0)
Ys1 = Fy(Xs0, Ys0)

dXs1 = Fx.grad(Xs0, Ys0)
dYs1 = Fy.grad(Xs0, Ys0)



from pylab import *

clf()
ioff()
for (x0,y0), (x1,y1) in zip(curve0, curve1):
    plot([x0,x1], [y0, y1], 'b')

for (x1,y1,dx1,dy1) in zip(Xs1, Ys1, dXs1, dYs1):
    plot([x1, x1+1*dx1[0]], [y1, y1+1*dy1[0]],'r')
    plot([x1, x1+1*dx1[1]], [y1, y1+1*dy1[1]],'g')
    
    a = np.array([dx1, dy1])
    q,r = qr(a)
    ox,oy = q[:,0], q[:,1]
    plot([x1, x1+ox[0]], [y1, y1+ox[1]],'r')
    plot([x1, x1+oy[0]], [y1, y1+oy[1]],'b')
    


#curve1a = np.c_[Fx(curve0[:,0]+randn(100)*.01,curve0[:,1]+randn(100)*.01), Fy(curve0[:,0]+randn(100)*.01, curve0[:,1]+randn(100)*.01)]
curve1a = np.c_[Fx(curve0[:,0],curve0[:,1]), Fy(curve0[:,0], curve0[:,1])]
plot(curve0[:,0],curve0[:,1],'bo')
plot(curve1[:,0],curve1[:,1],'bo')
plot(curve1a[:,0],curve1a[:,1],'gx',ms=8,mew=3)
show()
plot(Xs1.flatten(),Ys1.flatten(),'r.')

plt.axis("equal")