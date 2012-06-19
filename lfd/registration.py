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
        
    def fit(self, x_nd, y_nd, smoothing, angular_spring = 0, wt_n=None, verbose=True):
        self.n, self.d = n,d = x_nd.shape
        
        dists_tri = ssd.pdist(x_nd)
        K_nn = ssd.squareform(tps_kernel(dists_tri, d))
        
        P = np.c_[x_nd, np.ones((n,1))]
        
        
        if wt_n is None:
            wt_n = np.ones(n)

        reg_nn = smoothing * np.diag(1/(wt_n+.01))
        reg_ratio = angular_spring/smoothing
        A = np.r_[
            np.c_[K_nn + reg_nn, P],
            np.c_[P.T, reg_ratio * np.eye(d+1,d+1)]]
        A[-1,-1] = 0
        b = np.r_[y_nd, reg_ratio * np.eye(d+1,d)]

            
        coeffs = np.linalg.lstsq(A, b)[0]
        
        self.x_nd = x_nd
        self.w_nd = coeffs[:n,:]
        self.a_Dd = coeffs[n:,:]

        residual_cost = (wt_n[:,None] * ((y_nd - self.transform_points(x_nd))**2).sum(axis=1)).sum()
        curvature_cost = smoothing * np.trace(np.dot(self.w_nd.T, np.dot(K_nn, self.w_nd)))
        rotation_cost = angular_spring * ((np.eye(d) - self.a_Dd[:-1,:])**2).sum()
        self.cost = residual_cost + curvature_cost + rotation_cost
        if verbose:
            print "cost = residual + curvature + rotation"
            print " %.3g = %.3g + %.3g + %.3g"%(self.cost, residual_cost, curvature_cost, rotation_cost)
            print "affine transform:\n", self.a_Dd
        
    def transform_points(self, x_md):
        m,d = x_md.shape
        assert d==self.d
        dist_mn = ssd.cdist(x_md, self.x_nd)
        K_mn = tps_kernel(dist_mn,d)
        xhom_mD = np.c_[x_md, np.ones((m,1))]
        ypred_md = np.dot(K_mn, self.w_nd) + np.dot(xhom_mD, self.a_Dd)
        return ypred_md
    
    def transform_frames(self, x_md, rot_mdd, orthogonalize=True):
        m,d = x_md.shape
        assert d == self.d

        dist_mn = ssd.cdist(x_md, self.x_nd)
        grad_mdd = np.zeros((m,d,d))
        for i_d in xrange(d):
            diffs_mn = x_md[:,i_d][:,None] - self.x_nd[:,i_d][None,:]
            if self.d == 2:
                raise NotImplementedError
            elif self.d == 3:
                grad_mdd[:,:,i_d] = self.a_Dd[i_d,:][None,:] + np.dot(nan2zero(diffs_mn / dist_mn),self.w_nd[:,i_d])[:,None]
        newrot_mdd = (grad_mdd[:,:,:,None]* rot_mdd[:,None,:,:]).sum(axis=2)
        
        xhom_mD = np.c_[x_md, np.ones((m,1))]
        K_mn = tps_kernel(dist_mn, self.d)
        ypred_md = np.dot(K_mn, self.w_nd) + np.dot(xhom_mD, self.a_Dd)
        if orthogonalize: newrot_mdd =  orthogonalize3(newrot_mdd)
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

def tps_icp(x_nd, y_md, n_iter = 5, reg_init = .1, reg_final = .001, rad_init = .2, rad_final = .001, plotting = False, verbose=True):
    n,d = x_nd.shape
    regs = loglinspace(reg_init, reg_final, n_iter)
    rads = loglinspace(rad_init, rad_final, n_iter)
    f = ThinPlateSpline.identity(d)
    for i in xrange(n_iter):
        if plotting and i%plotting==0: 
            import matplotlib.pyplot as plt            
            plt.clf()
        xwarped_nd = f.transform_points(x_nd)
        # targ_nd = find_targets(x_nd, y_md, corr_opts = dict(r = rads[i], p = .1))
        corr_nm = calc_correspondence_matrix(xwarped_nd, y_md, r=rads[i], p=.2)
        
        wt_n = corr_nm.sum(axis=1)
        targ_nd = np.dot(corr_nm/wt_n[:,None], y_md)
        
        # if plotting:
        #     plot_correspondence(x_nd, targ_nd)
        
        f.fit(x_nd, targ_nd, regs[i], wt_n = wt_n, angular_spring = regs[i]*2, verbose=verbose)

        if plotting and i%plotting==0:
            plt.plot(x_nd[:,0], x_nd[:,1],'r.')
            plt.plot(y_md[:,0], y_md[:,1], 'b.')
            pred = f.transform_points(x_nd)
            plt.plot(pred[:,0], pred[:,1], 'g.')
            if f.d == 2:
                plot_warped_grid_2d(f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0))
                plt.ginput()
            elif f.d == 3:
                from lfd import warping
                from brett2.ros_utils import RvizWrapper,Marker
                from utils import conversions
                rviz = RvizWrapper.create()
                mins = x_nd.min(axis=0)
                maxes = x_nd.max(axis=0)
                mins[2] -= .1
                maxes[2] += .1
                handles = warping.draw_grid(rviz, f.transform_points, mins, maxes, 'base_footprint')
                orig_pose_array = conversions.array_to_pose_array(x_nd, "base_footprint")
                target_pose_array = conversions.array_to_pose_array(y_md, "base_footprint")
                warped_pose_array = conversions.array_to_pose_array(f.transform_points(x_nd), 'base_footprint')
                handles.append(rviz.draw_curve(orig_pose_array,rgba=(1,0,0,1),type=Marker.CUBE_LIST))
                handles.append(rviz.draw_curve(target_pose_array,rgba=(0,0,1,1),type=Marker.CUBE_LIST))
                handles.append(rviz.draw_curve(warped_pose_array,rgba=(0,1,0,1),type=Marker.CUBE_LIST))

        
    f.corr = corr_nm
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
        prob_nm /= (p*(n/m) + prob_nm.sum(axis=0))[None,:]  # cols sum to n/m
        prob_nm /= (p + prob_nm.sum(axis=1))[:,None] # rows sum to 1
        
    #print "row sums:", prob_nm.sum(axis=1)
    #print "col sums/ratio:", prob_nm.sum(axis=0)/(n/m)
    
    return prob_nm

def nan2zero(x):
    np.putmask(x, np.isnan(x), 0)
    return x

def orthogonalize3(mats_n33):
    x_n3 = mats_n33[:,:,0]
    y_n3 = mats_n33[:,:,1]
    z_n3 = mats_n33[:,:,2]
    
    xnew_n3 = math_utils.normr(x_n3)
    znew_n3 = math_utils.normr(np.cross(xnew_n3, y_n3))
    ynew_n3 = math_utils.normr(np.cross(znew_n3, xnew_n3))
    
    return np.concatenate([xnew_n3[:,:,None], ynew_n3[:,:,None], znew_n3[:,:,None]],2)
    

