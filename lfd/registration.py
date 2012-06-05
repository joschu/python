from __future__ import division
import lfd
import os
from utils import math_utils
import numpy as np
import scipy.spatial.distance as ssd


def tps_kernel(dist, dim):
    if dim == 1:
        return dist**3
    elif dim == 2:
        K = dist**2 * np.log(dist)
        np.putmask(K, dist==0, 0)
        return K
    elif dim == 3:
        return -dist
    else:
        raise NotImplementedError

class ThinPlateSpline(object):
    
    @staticmethod
    def identity(d):
        tps = ThinPlateSpline()
        tps.n = 0
        tps.d = d
        tps.x_nd = np.zeros((0,d))
        tps.w_nd = np.zeros((0,d))
        tps.a_Dd = np.eye(d+1,d)
        return tps    
        
    def fit(self, x_nd, y_nd, smoothing, wt_n=None):
        self.n, self.d = n,d = x_nd.shape
        
        dists_tri = ssd.pdist(x_nd)
        K_nn = ssd.squareform(tps_kernel(dists_tri, d))
        
        P = np.c_[x_nd, np.ones((n,1))]
        
        
        if wt_n is None:
            A = np.r_[
                np.c_[K_nn + smoothing * np.eye(n), P],
                np.c_[P.T, np.zeros((d+1,d+1))]]
            b = np.r_[y_nd, np.zeros((d+1,d))]
            
        else:
            alpha=10
            wt_n1 = np.asarray(wt_n).reshape(-1,1)
            A = np.r_[
                np.c_[wt_n1 * K_nn + smoothing * np.eye(n), wt_n1 * P],
                np.c_[P.T, alpha*np.eye(d+1)]]
            A[-1,-1] = 0
            b = np.r_[wt_n1 * y_nd, alpha*np.eye(d+1,d)]

            
        coeffs = np.linalg.lstsq(A, b)[0]
        
        self.x_nd = x_nd
        self.w_nd = coeffs[:n,:]
        self.a_Dd = coeffs[n:,:]
        print self.a_Dd
        
    def transform_points(self, x_md):
        m,d = x_md.shape
        assert d==self.d
        dist_mn = ssd.cdist(x_md, self.x_nd)
        K_mn = tps_kernel(dist_mn,d)
        xhom_mD = np.c_[x_md, np.ones((m,1))]
        ypred_md = np.dot(K_mn, self.w_nd) + np.dot(xhom_mD, self.a_Dd)
        return ypred_md
    
    def transform_frames(self, x_md, rot_mdd):
        m,d = x_md.shape
        assert d == self.d

        dist_mn = ssd.pdist(x_md, self.x_nd)
        grad_mdd = np.zeros((m,d,d))
        for i_d in xrange(d):
            diffs_mn = x_md[:,i_d][:,None] - x_md[:,i_d][None,:]
            if self.d == 2:
                grad_mdd[:,:,i_d] = self.a_Dd[i_d,:] + self.w_nd[:,i_d] * nan2zero(diffs_mn / dist_mn)
            elif self.d == 3:
                grad_mdd[:,:,i_d] = self.a_Dd[i_d,:] + self.w_nd[:,i_d] * nan2zero(diffs_mn / dist_mn)
        
        newrot_mdd = np.tensordot(rot_mdd, grad_mdd, axes=([2],[1]))

        xhom_mD = np.c_[x_md, np.ones((m,1))]
        K_mn = tps_kernel(dist_mn, self.d)
        ypred_md = np.dot(K_mn, self.w_nd) + np.dot(xhom_mD, self.a_Dd)
        
        return ypred_md, newrot_mdd

def plot_warped_grid_2d(f, mins, maxes):
    import matplotlib.pyplot as plt
    import matplotlib
    xmin, ymin = mins
    xmax, ymax = maxes
    ncoarse = 10
    nfine = 30
    xcoarse = np.linspace(xmin, xmax, ncoarse)
    ycoarse = np.linspace(ymin, ymax, ncoarse)
    xfine = np.linspace(xmin, xmax, nfine)
    yfine = np.linspace(ymin, ymax, nfine)
    
    lines = []
    
    for x in xcoarse:
        xy = np.zeros((nfine, 2))
        xy[:,0] = x
        xy[:,1] = yfine
        lines.append(f(xy))

    for y in ycoarse:
        xy = np.zeros((nfine, 2))
        xy[:,0] = xfine
        xy[:,1] = y
        lines.append(f(xy))        
    
    lc = matplotlib.collections.LineCollection(lines)
    ax = plt.gca()
    ax.add_collection(lc)
    plt.draw()
    
def plot_correspondence(x_nd, y_nd):
    lines = np.array(zip(x_nd, y_nd))
    import matplotlib.pyplot as plt
    import matplotlib
    lc = matplotlib.collections.LineCollection(lines)
    ax = plt.gca()
    ax.add_collection(lc)
    plt.draw()
    
def loglinspace(a,b,n):
    return np.exp(np.linspace(np.log(a),np.log(b),n))    

def tps_icp(x_nd, y_md, n_iter = 5, reg_init = .1, reg_final = .001, rad_init = .2, rad_final = .001, plotting = False):
    n,d = x_nd.shape
    regs = loglinspace(reg_init, reg_final, n_iter)
    rads = loglinspace(rad_init, rad_final, n_iter)
    f = ThinPlateSpline.identity(d)
    for i in xrange(n_iter):
        if i%plotting==0: 
            import matplotlib.pyplot as plt            
            plt.clf()
        xwarped_nd = f.transform_points(x_nd)
        # targ_nd = find_targets(x_nd, y_md, corr_opts = dict(r = rads[i], p = .1))
        corr_nm = calc_correspondence_matrix(xwarped_nd, y_md, r=rads[i], p=.25)
        
        wt_n = corr_nm.sum(axis=1)
        targ_nd = np.dot(corr_nm/wt_n[:,None], y_md)
        
        # if plotting:
        #     plot_correspondence(x_nd, targ_nd)
        
        f.fit(x_nd, targ_nd, regs[i], wt_n = wt_n)

        if i%plotting==0:
            plt.plot(x_nd[:,0], x_nd[:,1],'r.')
            plt.plot(y_md[:,0], y_md[:,1], 'b.')
            pred = f.transform_points(x_nd)
            plt.plot(pred[:,0], pred[:,1], 'g.')
            plot_warped_grid_2d(f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0))
            plt.ginput()
        
    return f

# matcher = None            
        
def find_targets(x_md, y_nd, corr_opts):
    # global matcher
    # if matcher is None:
    #     from optimization import matching
    #     M = matching.Matcher()
        
    corr_mn = calc_correspondence_matrix(x_md, y_nd, **corr_opts)
    # corr_mn = M.match(x_md, y_nd)
    # corr_mn = corr_mn / corr_mn.sum(axis=1)[:,None]
    return np.dot(corr_mn, y_nd)        
    
def calc_correspondence_matrix(x_nd, y_md, r, p, n_iter=20):
    n = x_nd.shape[0]
    m = y_md.shape[0]
    dist_nm = ssd.cdist(x_nd, y_md,'euclidean')
    prob_nm = np.exp(-dist_nm / r)
    for i in xrange(n_iter):
        prob_nm /= (n/m)*(p + prob_nm.sum(axis=0))[None,:]  # cols sum to n/m
        prob_nm /=    1 *(p + prob_nm.sum(axis=1))[:,None] # rows sum to 1
        
    print "row sums:", prob_nm.sum(axis=1)
    print "col sums/ratio:", prob_nm.sum(axis=0)/(n/m)
    
    return prob_nm

def nan2zero(x):
    np.putmask(x, np.isnan(x), 0)
    return x

def orthogonalize3(mats_n33):
    x_n3 = mats_n33[:,:,0]
    # y_n3 = mats_n33[:,:,1]
    z_n3 = mats_n33[:,:,2]
    
    xnew_n3 = math_utils.normr(x_n3)
    ynew_n3 = math_utils.normr(np.cross(z_n3, xnew_n3))
    znew_n3 = math_utils.normr(np.cross(xnew_n3, ynew_n3))
    
    return np.concatenate([xnew_n3[:,:,None], ynew_n3[:,:,None], znew_n3[:,:,None]],2)
    

