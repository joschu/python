from __future__ import division
import lfd
from lfd import registration, trajectory_library
import numpy as np
import os.path as osp
import scipy.io as sio

import matplotlib.pyplot as plt
from image_proc.clouds import voxel_downsample

    
def test_fitting():
    matobj = sio.loadmat(osp.join(osp.dirname(lfd.__file__), "matlab", "pointset_pair.mat"))
    
    points0 = matobj["xy1"]
    points1 = matobj["xy2"]
    
    tps = registration.ThinPlateSpline()
    tps.fit(points0, points1,.001)
    
    points0_tf = tps.transform_points(points0)
    
    plt.plot(points0[:,0], points0[:,1],'r.')
    plt.plot(points1[:,0], points1[:,1],'b.')
    plt.plot(points0_tf[:,0], points0_tf[:,1], 'g.')
    
    registration.plot_warped_grid_2d(tps.transform_points, points0.min(axis=0), points0.max(axis=0))
    
    
    
def test_icp():
    matobj = sio.loadmat(osp.join(osp.dirname(lfd.__file__), "matlab", "pointset_pair.mat"))
    
    points0 = matobj["xy1"][:-1]
    points1 = matobj["xy2"][:-1]
    
    tps = registration.tps_icp(points0, points1, plotting = True)
    
def test_registration_rope_images():
    
    library = trajectory_library.TrajectoryLibrary("lm.h5","read")
    


    plt.ion()
    segment_group = library.root["segments"]
    
    n_segs = len(segment_group.keys())
    n_rows = int(round(np.sqrt(n_segs)))
    n_cols = int(np.ceil(n_segs / n_rows))    
    print n_rows, n_cols
    
    sim_ropes = []
    for (i,(name, data)) in enumerate(segment_group.items()):
        plt.subplot(n_rows, n_cols, i+1)
        if "object_points" in segment_group:
            points_n3 = data["object_points"][0]
        else:
            print "rope field is deprecated. use object points"
            points_n3 = data["rope"][0]
        plt.plot(points_n3[:,1], points_n3[:,0],'.')
        plt.title(name)
    
        sim_ropes.append(points_n3)
        
    i=4
    rope = np.dot(np.squeeze(np.loadtxt("/home/joschu/Data/rope/rope%i.txt"%i)),np.diag([1,-1,-1]))    
    rope = voxel_downsample(rope,.03)
    
    #f = registration.tps_icp
    tps = registration.tps_icp(rope[:,:2], sim_ropes[i][:,:2],plotting=10,reg_init=1,reg_final=.01,n_iter=200)
    

if __name__ == "__main__":
    #test_icp()
    test_registration_rope_images()