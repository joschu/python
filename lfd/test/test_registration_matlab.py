from __future__ import division
import lfd
from lfd import registration_matlab, trajectory_library
import numpy as np
import mlabraw
import os.path as osp
import scipy.io as sio

import matplotlib.pyplot as plt
from image_proc import interactive_roi as roi
from image_proc import curves
import cv2
from image_proc.clouds import voxel_downsample

def test_mlab_3d():
    data = np.arange(24)
    x = data.reshape(2,3,4)
    
    handle = mlabraw.open("matlab -nodesktop -nodisplay -nojvm")
    
    registration_matlab.put3d(handle, "x", x)
    
    hopefully_x = registration_matlab.get3d(handle, "x")
    print hopefully_x
    assert np.allclose(x, hopefully_x)
    
def test_registration():
    matobj = sio.loadmat(osp.join(osp.dirname(lfd.__file__), "matlab", "pointset_pair.mat"))
    
    points0 = matobj["xy1"]
    points1 = matobj["xy2"]
    
    reg = registration_matlab.NonrigidRegistration(display=True)
    reg.fit_transformation_icp(points0, points1)
    
    mats = np.array([np.eye(2) for _ in points0])
    print "points1 - f(points0)"
    print points1 - reg.transform_points(points0)
    print "transformed poses:"
    print reg.transform_poses(points0, mats)


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
        
    
    #win_name = "get points"
    #cv2.namedWindow(win_name)
    #im = np.zeros((100,100), 'uint8')
    #im.fill(150)
    #xys = np.array(roi.get_polyline(im, win_name),float)/100
    #curve = curves.unif_resample(xys,100,tol=.1)
    print np.squeeze(np.loadtxt("/home/joschu/Data/rope/rope2.txt")).shape
    rope = np.dot(np.squeeze(np.loadtxt("/home/joschu/Data/rope/rope1.txt")),np.diag([1,-1,-1]))    
    rope = voxel_downsample(rope,.03)
    
    reg = registration_matlab.NonrigidRegistration(display=True)
    reg.fit_transformation_icp(rope[:,:2], sim_ropes[3][:,:2])    
    

