
import numpy as np
import itertools
from os.path import join, exists
from glob import glob
import h5py
import jds_utils.conversions as conv
import tps

LIBRARY_DIR = "/home/joschu/bulletsim/data/knots/library"

"""
Database layout
/demos (group)
   /demo0
   /demo1
   /demo2
   ...
/segments (group)
   /seg0
   /seg1
   /seg2
   ...
"""


class TrajectoryLibrary(object):
    def __init__(self, dbname, mode):
        if mode == "read": filemode = "r"
        elif  mode == "write": 
            if exists(join(LIBRARY_DIR, dbname)): filemode = "r+"
            else: filemode = "w"
        else:
            raise Exception("mode must be 'read' or 'write'")
        
        self.root = h5py.File(join(LIBRARY_DIR, dbname),filemode)
        if "segments" not in self.root: self.root.create_group("segments")
        if "demos" not in self.root: self.root.create_group("demos")
        
        # load all the trajectories
    def create_segments(self):
        "break demos into segments"
        if "segments" in self.root and len(self.root["segments"]) > 0: 
            del self.root["segments"]
            self.root.create_group("segments")
        for (name,traj) in self.root["demos"].items():
            break_times = get_break_times(traj)
            start_times = np.r_[0,break_times].astype('int')
            end_times = np.r_[break_times, len(traj)].astype('int')
            for (i,(start, end)) in enumerate(zip(start_times, end_times)):
                self.root["segments"].create_dataset("%s.%i"%(name, i), data=traj[start:end])

    def lookup_closest(self, obs):
        raise
            
    def get_closest_and_warp(self, obs):
        raise
    
    def add_demo(self, array, name=None):

        if name is None:
            n_prev_demos = len(self.root["demos"].keys())            
            name = "demo%i"%n_prev_demos

        self.root["demos"].create_dataset(name, data=array)    
    
    
def affine_residual(a, b):
    u,d,vh = np.linalg.svd(a,full_matrices=False)
    n_good_evs = (d > .001).sum()
    asmall = u[:,:n_good_evs]
    _,res,_,_ = np.linalg.lstsq(asmall,b)
    return res.sum()    
    
def connected_components_1d(array):
    ccs = []
    last_a = array[0]
    if last_a: cur_cc = [0]        
    else: cur_cc = []
    
    for i in xrange(1,len(array)):
        if array[i]:
            cur_cc.append(i)
        elif last_a:
            ccs.append(cur_cc)
            cur_cc = []
        last_a = array[i]
            
    if len(cur_cc) > 0: ccs.append(cur_cc)
    return ccs
            
def get_break_times(traj):
    "midpoints of successive changes in contact states"
    both_open = (traj["grab_l"] == -1) & (traj["grab_r"] == -1)
    both_open_ccs = connected_components_1d(both_open)
    
    if len(both_open_ccs)==1: return [both_open_ccs[0][0] + both_open_ccs[0][-1]//2]
    
    if both_open[0]: both_open_ccs.pop(0)
    if both_open[-1]: both_open_ccs.pop(-1)
    return [(cc[0] + cc[-1])//2 for cc in both_open_ccs]
    # if they both start out or end open, don't break those interval
    
def get_change_inds(arr):
    "return set of i such that arr[i] != arr[i-1]"
    return np.flatnonzero(arr[1:] != arr[:-1])+1

def fit_transform(pts0, pts1):
    
    xyz = np.concatenate([pts0, [[0,0,0]]],0)
    xyzp = np.concatenate([pts1, [[0,0,0]]],0)
    
    x,y,z = xyz.T
    xp,yp,zp = xyzp.T
    
    F = tps.TPS33(x,y,z, xp, yp, zp)

    return F
    
def transform_pose(xyz,quat,F):
    x,y,z = xyz
    xp, yp, zp = F.eval(np.r_[x],np.r_[y],np.r_[z]).T
    jac = F.grad(np.r_[x],np.r_[y],np.r_[z])[0]
    old_rot_mat = conv.quat2mat(quat)
    new_rot_mat = np.dot(jac, old_rot_mat)
    new_rot_mat_orth = np.linalg.qr(new_rot_mat)[0]
    new_quat = conv.mat2quat(new_rot_mat_orth)
    return np.r_[xp,yp,zp], new_quat


def transform_poses(xyzs,quats,F):
    x,y,z = xyzs.T
    xyzp = F.eval(x,y,z)
    jacs = F.grad(x,y,z)    
    
    new_quats = []
    for (quat,jac) in zip(quats,jacs):
        old_rot_mat = conv.quat2mat(quat)
        new_rot_mat = np.dot(jac, old_rot_mat)
        q,r = np.linalg.qr(new_rot_mat.T)
        new_rot_mat_orth = np.sign(np.diag(r))[:,None]* q.T
        new_quat = conv.mat2quat(new_rot_mat_orth)
        new_quats.append(new_quat)
    return xyzp, np.array(new_quats)


def transform_points(xyz,F):
    x,y,z = xyz.T
    return F.eval(x,y,z)
