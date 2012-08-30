import scipy.spatial.distance as ssd
import numpy as np
import scipy.optimize as opt
from copy import deepcopy
from numpy import sin, cos

def fit_score(src, targ, dist_param):
    "how good of a partial match is src to targ"
    sqdists = ssd.cdist(src, targ,'sqeuclidean')
    return -np.exp(-sqdists).sum()

def register_2d_multi(srcs, targs):
    "register srcs to targs"
    pass

class Transformation(object):
    n_params = 0
    def fit(self, x_nd, y_nd):
        abstract
    def transform_points(self, x_nd):
        abstract
    def transform_frames(self, x_nd, rot_nkk, orthogonalize=True):
        abstract

class TiltFree(Transformation):
    pass

class RigidPlanar(Transformation):
    n_params = 3
    tx = 0
    ty = 0
    angle = 0
    
    def set_params(self, params):
        self.tx, self.ty, self.angle = params        
    def get_params(self, params):
        return np.r_[self.tx, self.ty, self.angle]
        
    def fit(self, x_n3, y_n3):        
        
        trans_init = (y_n3.mean(axis=0) - x_n3.mean(axis=0))[:2]
        rot_inits = np.linspace(0, 2*np.pi, 6, endpoint=False)
        
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
        
        return np.dot(x_n3, rot_mat) + np.r_[self.tx, self.ty, 0][None,:]
            
            
        
            
        


class InterpolatedTransformation:
    "same interface as ThinPlateSpline"
    def fit(self, srcs, targs, transform_type = "tilt_free"):
        
        if transform_type == "tilt_free":
            TransformClass = TiltFree
        elif transform_type == "rigid_planar":
            TransformClass = RigidPlanar
        else:
            raise Exception("unrecognized transform class %s"%transform_type)
            
        
        assert len(srcs) == len(targs)
                                        
        self.transforms = []
        self.origins_kd = []
        
        
        for (src, targ) in zip(srcs, targs):
            transform = TransformClass()
            transform.fit(src, targ)
            self.transforms.append(transform)
            self.origins_kd.append(src.mean(axis=0))
            
        
    def transform_points(self, x_nd):
        dists_kn = ssd.cdist(self.origins_kd, x_nd)
        # weights_kn = 1/(dists_kn**2+1e-6)
        weights_kn = np.exp(-dists_kn**2/3**2)
        weights_kn /= weights_kn.sum(axis=0)[None,:]
        y_nd = np.zeros_like(x_nd)
        for k in xrange(dists_kn.shape[0]):
            # y_nd += weights_kn[k][:,None] * self.transforms[k].transform_points(x_nd)
            y_nd += weights_kn[k][:,None] * self.transforms[k].transform_points(x_nd)            
        return y_nd
