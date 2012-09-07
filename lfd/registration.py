"""
Register point clouds to each other
"""

from __future__ import division
from jds_utils import math_utils
import numpy as np
import scipy.spatial.distance as ssd
from numpy import cos, sin, pi
import scipy.optimize as opt
from copy import deepcopy
from lfd import warping
from brett2.ros_utils import Marker
from jds_utils import conversions
import matplotlib.pyplot as plt


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

class Transformation(object):
    n_params = 0
    def fit(self, x_nd, y_nd):
        raise NotImplementedError
    def transform_points(self, x_nd):
        raise NotImplementedError
    def transform_frames(self, x_nd, rot_nkk, orthogonalize=True):
        raise NotImplementedError

    def jacobian(self, x_d, epsilon=0.0001):
        x0 = np.asfarray(x_d)
        f0 = self.transform_points(x0)
        jac = np.zeros(len(x0), len(f0))
        dx = np.zeros(len(x0))
        for i in range(len(x0)):
            dx[i] = epsilon
            jac[i] = (self.transform_points(x0+dx) - f0) / epsilon
            dx[i] = 0.
        return jac.transpose()

    def approx_deriv(self, x_d, dx_d, dist=None):
        x_1d, dx_1d = np.asarray([x_d]), np.asarray([dx_d])
        if dist is None: dist = np.linalg.norm(dx_d)
        return (self.transform_points(x_1d)[0] - self.transform_points(x_1d + dx_1d)[0])/float(dist)

class ThinPlateSpline(Transformation):
    
    @staticmethod
    def identity(d):
        tps = ThinPlateSpline()
        tps.n = 0
        tps.d = d
        tps.x_nd = np.zeros((0,d))
        tps.w_nd = np.zeros((0,d))
        tps.a_Dd = np.eye(d+1,d)
        tps.id_interp = None
        tps.subtract_id = False
        return tps    

    def copy(self):
        tps = ThinPlateSpline()
        tps.n = self.n
        tps.d = self.d
        tps.x_nd = self.x_nd.copy()
        tps.w_nd = self.w_nd.copy()
        tps.a_Dd = self.a_Dd.copy()
        tps.id_interp = self.id_interp
        tps.subtract_id = self.subtract_id
        return tps
        
    def fit(self, x_nd, y_nd, smoothing=.1, angular_spring = 0, wt_n=None, verbose=True):
        """
        x_nd: source cloud
        y_nd: target cloud
        smoothing: penalize non-affine part
        angular_spring: penalize rotation
        wt_n: weight the points        
        """
        self.n, self.d = n,d = x_nd.shape
        
        dists_tri = ssd.pdist(x_nd)
        K_nn = ssd.squareform(tps_kernel(dists_tri, d))
        
        P = np.c_[x_nd, np.ones((n,1))]
        
        
        if wt_n is None:
            wt_n = np.ones(n)

        reg_nn = smoothing * np.diag(1/(wt_n+1e-6))
        reg_ratio = angular_spring/smoothing
        A = np.r_[
            np.c_[K_nn + reg_nn, P],
            np.c_[P.T, reg_ratio * np.eye(d+1,d+1)]]
        A[-1,-1] = 0
        b = np.r_[y_nd, reg_ratio * np.eye(d+1,d)]

        coeffs = np.linalg.lstsq(A, b)[0]
        self.w_nd = coeffs[:n,:]
        self.a_Dd = coeffs[n:,:]
        rotation_cost = angular_spring * ((np.eye(d) - self.a_Dd[:-1,:])**2).sum()
        
        self.x_nd = x_nd

        residual_cost = (wt_n[:,None] * ((y_nd - self.transform_points(x_nd))**2).sum(axis=1)).sum()
        curvature_cost = smoothing * np.trace(np.dot(self.w_nd.T, np.dot(K_nn, self.w_nd)))
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
        if self.id_interp is not None:
            ypred_md = (1.-self.id_interp)*x_md + self.id_interp*ypred_md
        if self.subtract_id:
            ypred_md -= x_md
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
        if self.id_interp is not None:
            ypred_md = (1.-self.id_interp)*x_md + self.id_interp*ypred_md
        if self.subtract_id:
            ypred_md -= x_md
        return ypred_md, newrot_mdd

    def interp_with_identity(self, a):
        tps = self.copy()
        tps.id_interp = a
        return tps

    def subtract_identity(self):
        tps = self.copy()
        tps.subtract_id = True
        return tps

class CompositeTransformation(Transformation):
    def __init__(self, init_fn):
      self.fns = [init_fn]

    def compose_with(self, f):
      self.fns.append(f)

    def get_last_fn(self):
      return self.fns[-1]

    def fit(self, x_nd, y_nd):
      assert False

    def transform_points(self, x_md):
      for f in self.fns:
        x_md = f.transform_points(x_md)
      return x_md

    def transform_frames(self, x_md, rot_mdd, orthogonalize=True):
      for f in self.fns:
        x_md, rot_mdd = f.transform_frames(x_md, rot_mdd, orthogonalize)
      return x_md, rot_mdd

class ThinPlateSplineFixedRot(ThinPlateSpline):
    
    def __init__(self, rot):
        ThinPlateSpline.__init__(self)
        assert rot.ndim == 2 and rot.shape[0] == rot.shape[1]
        self.n = 0
        self.d = rot.shape[0]
        self.x_nd = np.zeros((0,self.d))
        self.w_nd = np.zeros((0,self.d))
        self.a_Dd = np.eye(self.d+1,self.d)
        self.a_Dd[:self.d, :] = rot
    
    def fit(self, x_nd, y_nd, smoothing=.1, wt_n=None, verbose=True):
        """
        x_nd: source cloud
        y_nd: target cloud
        smoothing: penalize non-affine part
        angular_spring: penalize rotation
        wt_n: weight the points        
        """
        self.n, self.d = n,d = x_nd.shape
        
        dists_tri = ssd.pdist(x_nd)
        K_nn = ssd.squareform(tps_kernel(dists_tri, d))
        
        
        
        if wt_n is None:
            wt_n = np.ones(n)

        reg_nn = smoothing * np.diag(1/(wt_n+1e-6))
        A = np.r_[
            np.c_[K_nn + reg_nn,       np.ones((n,1))],
            np.c_[np.ones((1,n)),      0]]
        b = np.r_[y_nd - np.dot(x_nd, self.a_Dd[:d,:]), np.zeros((1,d))]

        
        coeffs = np.linalg.lstsq(A, b)[0]
        self.w_nd = coeffs[:n,:]
        self.a_Dd[-1] = coeffs[n,:]
        rotation_cost = 0
        
        self.x_nd = x_nd

        residual_cost = (wt_n[:,None] * ((y_nd - self.transform_points(x_nd))**2).sum(axis=1)).sum()
        curvature_cost = smoothing * np.trace(np.dot(self.w_nd.T, np.dot(K_nn, self.w_nd)))
        self.cost = residual_cost + curvature_cost + rotation_cost
        if verbose:
            print "cost = residual + curvature + rotation"
            print " %.3g = %.3g + %.3g + %.3g"%(self.cost, residual_cost, curvature_cost, rotation_cost)
            print "affine transform:\n", self.a_Dd    

def plot_warped_grid_2d(f, mins, maxes, grid_res=None, flipax = True):
    import matplotlib.pyplot as plt
    import matplotlib
    xmin, ymin = mins
    xmax, ymax = maxes
    ncoarse = 10
    nfine = 30
    
    if grid_res is None:
        xcoarse = np.linspace(xmin, xmax, ncoarse)
        ycoarse = np.linspace(ymin, ymax, ncoarse)
    else:
        xcoarse = np.arange(xmin, xmax, grid_res)
        ycoarse = np.arange(ymin, ymax, grid_res)
    xfine = np.linspace(xmin, xmax, nfine)
    yfine = np.linspace(ymin, ymax, nfine)
    
    lines = []
    
    sgn = -1 if flipax else 1
    
    for x in xcoarse:
        xy = np.zeros((nfine, 2))
        xy[:,0] = x
        xy[:,1] = yfine
        lines.append(f(xy)[:,::sgn])

    for y in ycoarse:
        xy = np.zeros((nfine, 2))
        xy[:,0] = xfine
        xy[:,1] = y
        lines.append(f(xy)[:,::sgn])        
    
    lc = matplotlib.collections.LineCollection(lines,colors='gray',lw=2)
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
    "n numbers between a to b (inclusive) with constant ratio between consecutive numbers"
    return np.exp(np.linspace(np.log(a),np.log(b),n))    


def tps_rpm_multi(source_clouds, targ_clouds, *args,**kw):
    """
    Given multiple source clouds and corresponding target clouds, solve for global transformation that matches them.
    Right now, we just concatenate them all. Eventually, we'll do something more sophisticated, e.g. initially match each
    source cloud to each target cloud, then use those correspondences to initialize tps-rpm on concatenated clouds.    
    """
    x_nd = np.concatenate([np.asarray(cloud).reshape(-1,3) for cloud in source_clouds], 0)
    y_md = np.concatenate([np.asarray(cloud).reshape(-1,3) for cloud in targ_clouds], 0)
    from jds_image_proc.clouds import voxel_downsample
    x_nd = voxel_downsample(x_nd,.02)
    y_md = voxel_downsample(y_md,.02)
    return tps_rpm(x_nd, y_md, *args,**kw)


class Globals:
    handles = []
    rviz = None
    @staticmethod
    def setup():
        if Globals.rviz is None:
            from brett2.ros_utils import RvizWrapper
            Globals.rviz = RvizWrapper.create()

def tps_rpm_with_overlap_control(x_nd, y_md, ctl_pts, jmin=0.3, **kwargs):
  def calculate_overlap(f, jmin, ctl_pts):
    def solve_interp_factor(T, pt, jmin, jac_eps=0.0001):
        # returns a s.t. det(jacobian((1-a)*I + a*T)) = jmin
        # from __future__ import division VERY IMPORTANT
        # f_x is df/dx, f_y is df/dy, etc., where f, g, and h are the components of T.
        f_x, g_x, h_x = T.approx_deriv(pt, [jac_eps, 0, 0], jac_eps)
        f_y, g_y, h_y = T.approx_deriv(pt, [0, jac_eps, 0], jac_eps)
        f_z, g_z, h_z = T.approx_deriv(pt, [0, 0, jac_eps], jac_eps)
        print 'jacobian: '
        print f_x, g_x, h_x
        print f_y, g_y, h_y
        print f_z, g_z, h_z
        # I did this by hand
        r0 = ((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))/((-1/2 - 3**(1/2)*1j/2)*((-jmin + 1)/(2*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) + (((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))**3 + ((-jmin + 1)/(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + 2*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**2/4)**(1/2) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(6*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**(1/3)) - (-1/2 - 3**(1/2)*1j/2)*((-jmin + 1)/(2*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) + (((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))**3 + ((-jmin + 1)/(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + 2*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**2/4)**(1/2) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(6*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**(1/3) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x))
        r1 = ((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))/((-1/2 + 3**(1/2)*1j/2)*((-jmin + 1)/(2*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) + (((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))**3 + ((-jmin + 1)/(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + 2*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**2/4)**(1/2) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(6*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**(1/3)) - (-1/2 + 3**(1/2)*1j/2)*((-jmin + 1)/(2*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) + (((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))**3 + ((-jmin + 1)/(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + 2*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**2/4)**(1/2) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(6*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**(1/3) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x))
        r2 = ((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))/((-jmin + 1)/(2*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) + (((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))**3 + ((-jmin + 1)/(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + 2*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**2/4)**(1/2) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(6*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**(1/3) - ((-jmin + 1)/(2*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) + (((f_x + g_y + h_z)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**2/(9*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2))**3 + ((-jmin + 1)/(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + 2*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**2/4)**(1/2) - (f_x + g_y + h_z)*(f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(6*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**2) + (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)**3/(27*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x)**3))**(1/3) - (f_x*g_y + f_x*h_z - f_y*g_x - f_z*h_x + g_y*h_z - g_z*h_y)/(3*(f_x*g_y*h_z - f_x*g_z*h_y - f_y*g_x*h_z + f_y*g_z*h_x + f_z*g_x*h_y - f_z*g_y*h_x))
        print 'roots', (r0, r1, r2)
        return [r.real for r in (r0, r1, r2) if r.imag == 0 and r.real > 0 and r.real <= 1] # only look at real solutions in (0, 1]

    # choose a so that |jac((1-a)*I + a*f)| > jmin on ctl_pts
    a = 1
    for pt in ctl_pts:
      rs = solve_interp_factor(f, pt, jmin)
      chosen_r = 1 if len(rs) == 0 else min(rs)
      a = min(a, chosen_r)
    return a

  orig_x_nd = x_nd.copy()
  t = CompositeTransformation(ThinPlateSpline.identity(x_nd.shape[1]))
  n_iter = 10
  tps_rpm_kwargs = kwargs.copy(); tps_rpm_kwargs['plotting'] = False
  for i in range(n_iter):
    f = tps_rpm(x_nd, y_md, **tps_rpm_kwargs)
    if i == n_iter - 1:
      t.compose_with(f) # on the last iteration, force convergence
    else:
      a = calculate_overlap(f, jmin, t.transform_points(np.asarray(ctl_pts)))
      print 'Smoothing out overlap: a:', a, 'iter:', i
      t.compose_with(f.interp_with_identity(a))
      if a == 1: break # converged
    x_nd = t.get_last_fn().transform_points(x_nd)

# orig_x_nd = x_nd.copy()
# t = CompositeTransformation(ThinPlateSpline.identity(x_nd.shape[1]))
# n_iter = 100
# tps_rpm_kwargs = kwargs.copy(); tps_rpm_kwargs['plotting'] = False
# for i in range(n_iter):
#   f = tps_rpm(x_nd, y_md, **tps_rpm_kwargs)
#   if i == n_iter - 1:
#     t.compose_with(f) # on the last iteration, force convergence
#   else:
#     #a = calculate_overlap(f, jmin, t.transform_points(np.asarray(ctl_pts)))
#     a = 0.02
#     print 'Smoothing out overlap: a:', a, 'iter:', i
#     t.compose_with(f.interp_with_identity(a))
#     if a == 1: break # converged
#   x_nd = t.get_last_fn().transform_points(x_nd)

    if 'plotting' in kwargs and kwargs['plotting']:
      from lfd import warping
      from brett2.ros_utils import Marker
      from jds_utils import conversions
      Globals.setup()
      mins = orig_x_nd.min(axis=0)
      maxes = orig_x_nd.max(axis=0)
      mins -= .1; maxes += .1
      Globals.handles = warping.draw_grid(Globals.rviz, t.transform_points, mins, maxes, 'base_footprint', xres=.01, yres=.01, zres=-1)
      orig_pose_array = conversions.array_to_pose_array(orig_x_nd, "base_footprint")
      target_pose_array = conversions.array_to_pose_array(y_md, "base_footprint")
      warped_pose_array = conversions.array_to_pose_array(t.transform_points(orig_x_nd), 'base_footprint')
      Globals.handles.append(Globals.rviz.draw_curve(orig_pose_array,rgba=(1,0,0,1),type=Marker.CUBE_LIST))
      Globals.handles.append(Globals.rviz.draw_curve(target_pose_array,rgba=(0,0,1,1),type=Marker.CUBE_LIST))
      Globals.handles.append(Globals.rviz.draw_curve(warped_pose_array,rgba=(0,1,0,1),type=Marker.CUBE_LIST))

  return t

def tps_rpm(x_nd, y_md, n_iter = 5, reg_init = .1, reg_final = .001, rad_init = .2, rad_final = .001, plotting = False, verbose=True, f_init = None):
    n,d = x_nd.shape
    regs = loglinspace(reg_init, reg_final, n_iter)
    rads = loglinspace(rad_init, rad_final, n_iter)
    f = ThinPlateSpline.identity(d)
    for i in xrange(n_iter):
        if f.d==2 and i%plotting==0:
            import matplotlib.pyplot as plt
            plt.clf()
        if i==0 and f_init is not None:
            xwarped_nd = f_init(x_nd)
            print xwarped_nd.max(axis=0)
        else:
            xwarped_nd = f.transform_points(x_nd)
        # targ_nd = find_targets(x_nd, y_md, corr_opts = dict(r = rads[i], p = .1))
        corr_nm = calc_correspondence_matrix(xwarped_nd, y_md, r=rads[i], p=.2)

        wt_n = corr_nm.sum(axis=1)
        # TODO(john): fix weight zero problem
        targ_nd = np.dot(corr_nm/(wt_n[:,None]+1e-9), y_md)

        # if plotting:
        #     plot_correspondence(x_nd, targ_nd)
        #print "warning: changed angular spring!"        
        f.fit(x_nd, targ_nd, regs[i], wt_n = wt_n, angular_spring = regs[i]*200	, verbose=verbose)

        if plotting and i%plotting==0:
            if f.d==2:
                plt.plot(x_nd[:,1], x_nd[:,0],'r.')
                plt.plot(y_md[:,1], y_md[:,0], 'b.')
            pred = f.transform_points(x_nd)
            if f.d==2:
                plt.plot(pred[:,1], pred[:,0], 'g.')
            if f.d == 2:
                plot_warped_grid_2d(f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0))
                plt.ginput()
            elif f.d == 3:
                Globals.setup()

                mins = x_nd.min(axis=0)
                maxes = x_nd.max(axis=0)
#                mins[2] -= .1
#                maxes[2] += .1
                mins -= .1; maxes += .1
                Globals.handles = warping.draw_grid(Globals.rviz, f.transform_points, mins, maxes, 'base_footprint', xres=.01, yres=.01, zres=-1)
                orig_pose_array = conversions.array_to_pose_array(x_nd, "base_footprint")
                target_pose_array = conversions.array_to_pose_array(y_md, "base_footprint")
                warped_pose_array = conversions.array_to_pose_array(f.transform_points(x_nd), 'base_footprint')
                Globals.handles.append(Globals.rviz.draw_curve(orig_pose_array,rgba=(1,0,0,1),type=Marker.CUBE_LIST))
                Globals.handles.append(Globals.rviz.draw_curve(target_pose_array,rgba=(0,0,1,1),type=Marker.CUBE_LIST))
                Globals.handles.append(Globals.rviz.draw_curve(warped_pose_array,rgba=(0,1,0,1),type=Marker.CUBE_LIST))

    f.corr = corr_nm
    return f

def tps_rpm_zrot(x_nd, y_md, n_iter = 5, reg_init = .1, reg_final = .001, rad_init = .2, rad_final = .001, plotting = False, verbose=True):
    n,d = x_nd.shape
    regs = loglinspace(reg_init, reg_final, n_iter)
    rads = loglinspace(rad_init, rad_final, n_iter)
    zrots = np.linspace(-np.pi/3, pi/3, 7)

    displacement = np.median(y_md,axis=0) - np.median(x_nd, axis=0) 
    
    costs = []
    
    if plotting:
        plotter = FuncPlotter()
        
    zrot2func = {}
    def fit0(zrot):
        f = ThinPlateSplineFixedRot(np.array([[cos(zrot), sin(zrot), 0], [-sin(zrot), cos(zrot), 0], [0, 0, 1]]))        
        f.a_Dd[3, :3] = displacement
        
        for i in xrange(n_iter):
    
            if f.d==2 and i%plotting==0: 
                import matplotlib.pyplot as plt            
                plt.clf()
    
            xwarped_nd = f.transform_points(x_nd)
            corr_nm = calc_correspondence_matrix(xwarped_nd, y_md, r=rads[i], p=.2)
            
            wt_n = corr_nm.sum(axis=1)
            targ_nd = np.dot(corr_nm/wt_n[:,None], y_md)
            f.fit(x_nd, targ_nd, regs[i], wt_n = wt_n, verbose=verbose)
    
            if plotting and i%plotting==0:
                plot_orig_and_warped_clouds(f, x_nd, y_md)

        print "zrot: %.3e, cost: %.3e,  meancorr: %.3e"%(zrot, f.cost, corr_nm.mean())
        cost = abs(zrot)/6 + f.cost
        if plotting: plotter.addpt(zrot, cost)
        zrot2func[zrot] = f
        return cost
    
    
    costs = [fit0(zrot) for zrot in zrots]
    i_best = np.argmin(costs)

    import scipy.optimize as so
    zspacing = zrots[1] - zrots[0]  
    print (zrots[i_best] - zspacing, zrots[i_best], zrots[i_best] + zspacing)
    zrot_best = so.golden(fit0, brack = (zrots[i_best] - zspacing, zrots[i_best], zrots[i_best] + zspacing), tol = .15)
    f_best = zrot2func[zrot_best]
    return f_best
    
 
class FuncPlotter(object):
    def __init__(self, fignum = 1):
        self.fignum = fignum
        plt.figure(fignum)
        plt.clf()
        self.xs = []
        self.ys = []
    def addpt(self, x, y):
        self.xs.append(x)
        self.ys.append(y)
        self.plotme()
    def plotme(self):
        plt.figure(self.fignum)
        plt.clf()
        plt.plot(self.xs, self.ys,'x')
        plt.draw()
 
def plot_orig_and_warped_clouds(f, x_nd, y_md): 
    if f.d==2:
        import matplotlib.pyplot as plt
        plt.plot(x_nd[:,1], x_nd[:,0],'r.')
        plt.plot(y_md[:,1], y_md[:,0], 'b.')
    pred = f.transform_points(x_nd)
    if f.d==2:
        plt.plot(pred[:,1], pred[:,0], 'g.')
    if f.d == 2:
        plot_warped_grid_2d(f.transform_points, x_nd.min(axis=0), x_nd.max(axis=0))
        plt.ginput()
    elif f.d == 3:
        
        Globals.setup()

        mins = x_nd.min(axis=0)
        maxes = x_nd.max(axis=0)
        mins[2] -= .1
        maxes[2] += .1
        Globals.handles = warping.draw_grid(Globals.rviz, f.transform_points, mins, maxes, 'base_footprint', xres=.1, yres=.1)
        orig_pose_array = conversions.array_to_pose_array(x_nd, "base_footprint")
        target_pose_array = conversions.array_to_pose_array(y_md, "base_footprint")
        warped_pose_array = conversions.array_to_pose_array(f.transform_points(x_nd), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(orig_pose_array,rgba=(1,0,0,1),type=Marker.CUBE_LIST))
        Globals.handles.append(Globals.rviz.draw_curve(target_pose_array,rgba=(0,0,1,1),type=Marker.CUBE_LIST))
        Globals.handles.append(Globals.rviz.draw_curve(warped_pose_array,rgba=(0,1,0,1),type=Marker.CUBE_LIST))

        
def find_targets(x_md, y_nd, corr_opts):
    """finds correspondence matrix, and then for each point in source cloud,
    find the weighted average of its "partners" in the target cloud"""

    corr_mn = calc_correspondence_matrix(x_md, y_nd, **corr_opts)
    # corr_mn = M.match(x_md, y_nd)
    # corr_mn = corr_mn / corr_mn.sum(axis=1)[:,None]
    return np.dot(corr_mn, y_nd)        
    
def calc_correspondence_matrix(x_nd, y_md, r, p, n_iter=20):
    "sinkhorn procedure. see tps-rpm paper"
    n = x_nd.shape[0]
    m = y_md.shape[0]
    dist_nm = ssd.cdist(x_nd, y_md,'euclidean')
    prob_nm = np.exp(-dist_nm / r)
    for _ in xrange(n_iter):
        prob_nm /= (p*(n/m) + prob_nm.sum(axis=0))[None,:]  # cols sum to n/m
        prob_nm /= (p + prob_nm.sum(axis=1))[:,None] # rows sum to 1
        
    #print "row sums:", prob_nm.sum(axis=1)
    #print "col sums/ratio:", prob_nm.sum(axis=0)/(n/m)
    
    return prob_nm

def nan2zero(x):
    np.putmask(x, np.isnan(x), 0)
    return x

def orthogonalize3(mats_n33):
    "turns each matrix into a rotation"

    x_n3 = mats_n33[:,:,0]
    y_n3 = mats_n33[:,:,1]
    # z_n3 = mats_n33[:,:,2]
    
    xnew_n3 = math_utils.normr(x_n3)
    znew_n3 = math_utils.normr(np.cross(xnew_n3, y_n3))
    ynew_n3 = math_utils.normr(np.cross(znew_n3, xnew_n3))
    
    return np.concatenate([xnew_n3[:,:,None], ynew_n3[:,:,None], znew_n3[:,:,None]],2)
    
def fit_score(src, targ, dist_param):
    "how good of a partial match is src to targ"
    sqdists = ssd.cdist(src, targ,'sqeuclidean')
    return -np.exp(-sqdists/dist_param**2).sum()


class Rigid2d(Transformation):
    n_params = 3
    tx = 0
    ty = 0
    angle = 0

    def set_params(self, params):
        self.tx, self.ty, self.angle = params        
    def get_params(self):
        return np.r_[self.tx, self.ty, self.angle]

    def fit(self, x_n3, y_n3):        

        trans_init = (y_n3.mean(axis=0) - x_n3.mean(axis=0))[:2]
        rot_inits = [0]

        self_copy = deepcopy(self)
        def f(params):
            self_copy.set_params(params)
            xmapped_n3 = self_copy.transform_points(x_n3)
            return fit_score(xmapped_n3, y_n3,.5)


        vals_params = []
        for rot_init in rot_inits:
            opt_params, opt_val, _, _, _ = opt.fmin_cg(f, np.r_[trans_init, rot_init],full_output=True)
            vals_params.append((opt_val, opt_params))


        best_val, best_params = min(vals_params, key = lambda x:x[0])
        print "best_params:", best_params
        self.set_params(best_params)
        self.objective = best_val

    def transform_points(self, x_n3):

        a = self.angle
        rot_mat = np.array([[cos(a), sin(a), 0],
                            [-sin(a), cos(a), 0],
                            [0,     0,      1]])

        return np.dot(x_n3, rot_mat.T) + np.r_[self.tx, self.ty, 0][None,:]
        
        
    def transform_frames(self, x_n3, rot_n33, orthogonalize=True):
        newx_n3 = self.transform_points(x_n3)
        
        a = self.angle
        j = np.array([[cos(a), sin(a), 0],
                      [-sin(a), cos(a), 0],
                      [0,     0,      1]])        
        
        newrot_n33 = np.array([np.dot(j, rot) for rot in rot_n33])
        return newx_n3, newrot_n33


class Translation2d(Transformation):
    n_params = 2
    tx = 0
    ty = 0

    def set_params(self, params):
        self.tx, self.ty = params        
    def get_params(self, params):
        return np.r_[self.tx, self.ty]

    def fit(self, x_n3, y_n3):        

        trans_init = (y_n3.mean(axis=0) - x_n3.mean(axis=0))[:2]

        self_copy = deepcopy(self)
        def f(params):
            self_copy.set_params(params)
            xmapped_n3 = self_copy.transform_points(x_n3)
            return fit_score(xmapped_n3, y_n3,.5)


        # vals_params = []
        # for rot_init in rot_inits:
        #     opt_params, opt_val, _, _, _ = opt.fmin_cg(f, np.r_[trans_init],full_output=True)
        #     vals_params.append((opt_val, opt_params))
        # 
        # 
        # best_val, best_params = min(vals_params, key = lambda x:x[0])
        
        best_params, best_val, _,_,_ = opt.fmin_cg(f, np.r_[trans_init], full_output=True)
        
        print "best_params:", best_params
        self.set_params(best_params)
        self.objective = best_val

    def transform_points(self, x_n3):

        return x_n3 + np.r_[self.tx, self.ty, 0][None,:]


    def transform_frames(self, x_n3, rot_n33, orthogonalize=True):
        newx_n3 = self.transform_points(x_n3)

        return newx_n3, rot_n33
        
        
