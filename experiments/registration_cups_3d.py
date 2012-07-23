from __future__ import division
from pylab import *
import scipy.spatial.distance as ssd
from utils.conversions import rod2mat
from lfd.registration import plot_warped_grid_2d, tps_rpm
clf()
ion()


cup0, cup1 = np.load("/home/joschu/python/lfd/data/images/pour0_segs.npy")
im_src = imread("/home/joschu/Data/misc/mugs_src.png")[:,:,0]
uvs_src = np.c_[np.nonzero(im_src==0)]*SCALE
us_src, vs_src = uvs_src.T

im_tgt = imread("/home/joschu/Data/misc/mugs_tgt.png")[:,:,0]
uvs_tgt = np.c_[np.nonzero(im_tgt==0)]*SCALE
us_tgt, vs_tgt = uvs_tgt.T


plot(vs_src, us_src, 'r.')
plot(vs_tgt, us_tgt, 'b.')


mug0src = uvs_src[vs_src < 100*SCALE]
mug1src = uvs_src[vs_src >= 100*SCALE]


mug0tgt = uvs_tgt[us_tgt < 100*SCALE]
mug1tgt = uvs_tgt[us_tgt >= 100*SCALE]


z0src = mug0src.mean(axis=0)
z1src = mug1src.mean(axis=0)

z0tgt = mug0tgt.mean(axis=0)
z1tgt = mug1tgt.mean(axis=0)

def rot_mat(theta):
    return np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])

def match_score(pts0, pts1, dist_param):
    dists = ssd.cdist(pts0, pts1,'sqeuclidean')
    return dists.min(axis=1).sum() + dists.min(axis=0).sum()
def local_trans(x, zsrc, ztgt, theta):
    return (ztgt - zsrc) + np.dot(x - zsrc[None,:],rot_mat(theta).T)

def cost(param):
    z0tgt = param[:2]
    theta0 = param[2]
    z1tgt = param[3:5]
    theta1 = param[5]
    
    mug0mapped = local_trans(mug0src, z0src, z0tgt, theta0)
    mug1mapped = local_trans(mug1src, z1src, z1tgt, theta1)
    
    e_match =  match_score(mug0mapped, mug0tgt,5) + match_score(mug1mapped, mug1tgt, 5)
    e_bend = 10*(np.linalg.norm(local_trans(z1src, z0src, z0tgt, theta0) - z1tgt)**2 + 
                 np.linalg.norm(local_trans(z0src, z1src, z1tgt, theta1) - z0tgt)**2)
    return e_match + e_bend

class Plotter:
    counter = 0
    green_plot = None
    def callback(self, xk):
        
        s_z0tgt = xk[:2]
        s_theta0 = xk[2]
        s_z1tgt = xk[3:5]
        s_theta1 = xk[5]
        
        mug0mapped = local_trans(mug0src, z0src, s_z0tgt, s_theta0)
        mug1mapped = local_trans(mug1src, z1src, s_z1tgt, s_theta1)
        mugsmapped = np.r_[mug0mapped, mug1mapped]        
        
        self.counter += 1
        if self.counter % 10 == 0:
            print "plotting"
            if self.green_plot is not None:
                ax = gca()
                ax.lines.remove(self.green_plot)
                ax.collections = []
            self.green_plot, = plot(mugsmapped[:,1], mugsmapped[:,0], 'g.')
            def warp_fn(x):
                return warp(x, s_z0tgt, s_z1tgt, s_theta0, s_theta1)
            plot_warped_grid_2d(warp_fn, uvs_src.min(axis=0), uvs_src.max(axis=0))
            
            
            draw()
        else:
            print "not plotting"
                
import scipy.optimize as so

plotter = Plotter()

def warp(x, z0tgt, z1tgt, theta0, theta1):
    x = np.atleast_2d(x)
    dists = ssd.cdist(np.array([z0src, z1src]), x)
    wts = np.exp(-dists/.01)
    wts = wts / wts.sum(axis=0)[None,:]
    return wts[0][:,None] * local_trans(x, z0src, z0tgt, theta0) + \
           wts[1][:,None] * local_trans(x, z1src, z1tgt, theta1)


soln = so.fmin_cg(cost, np.r_[z0tgt, 0, z1tgt, 0],callback=plotter.callback)


def warp_fn(x, params):
    s_z0tgt = params[:2]
    s_theta0 = params[2]
    s_z1tgt = params[3:5]
    s_theta1 = params[5]

    return warp(x, s_z0tgt, s_z1tgt, s_theta0, s_theta1)



   
plt.figure(2)
tps_rpm(uvs_src, uvs_tgt, n_iter = 100, reg_init=1, reg_final = .1, plotting=4, rad_init=.01, rad_final = .001, f_init = lambda x: warp_fn(x, soln))





#plt.plot(mug0src[:,1], mug0src[:,0],'cx')
#plt.plot(mug0tgt[:,1], mug0tgt[:,0],'cx')
show()