from __future__ import division
import sys
import lfd
from lfd import registration, trajectory_library
import numpy as np
import os.path as osp
import scipy.io as sio

import matplotlib.pyplot as plt
from jds_image_proc.clouds import voxel_downsample

def test_degenerate_fitting():
    points0 = np.array([[0,0]])
    points1 = np.array([[1,1]])
    tps = registration.ThinPlateSpline()
    tps.fit(points0, points1, smoothing=.1, angular_spring=.1)
    
def test_fitting():
    matobj = sio.loadmat(osp.join(osp.dirname(lfd.__file__), "matlab", "pointset_pair.mat"))
    
    points0 = matobj["xy1"]
    points1 = matobj["xy2"]
    
    tps = registration.ThinPlateSpline()
    tps.fit(points0, points1,.001,.001)
    
    points0_tf = tps.transform_points(points0)
    
    plt.plot(points0[:,0], points0[:,1],'r.')
    plt.plot(points1[:,0], points1[:,1],'b.')
    plt.plot(points0_tf[:,0], points0_tf[:,1], 'g.')
    
    registration.plot_warped_grid_2d(tps.transform_points, points0.min(axis=0), points0.max(axis=0))
    
def test_multi_fitting():
    library = trajectory_library.TrajectoryLibrary(osp.join(osp.dirname(lfd.__file__),"data/lm.h5"),"read")
    drawn_points = np.load(osp.join(osp.dirname(lfd.__file__),"data/should_be_demo01.txt.npy"))
    
    best_cost = np.inf
    best_f = None
    best_name = None
    for (seg_name,trajectory) in library.root["segments"].items():
        #f = registration.ThinPlateSpline()
        #f.fit(trajectory["rope"][0], 
              #drawn_points, 1e-2,1e-2)
        f = registration.tps_rpm(np.asarray(trajectory["rope"][0][::-1]), drawn_points, plotting = False)
        print "seg_name: %s. cost: %s"%(seg_name, f.cost)
        if f.cost < best_cost:
            best_cost = f.cost
            best_f = f
            best_name = seg_name
    seg_name = best_name
    print seg_name
    
def test_icp():
    matobj = sio.loadmat(osp.join(osp.dirname(lfd.__file__), "matlab", "pointset_pair.mat"))
    
    points0 = matobj["xy1"][:-1]
    points1 = matobj["xy2"][:-1]
    
    tps = registration.tps_rpm(points0, points1, plotting = True)
    
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
        
    i=int(sys.argv[1])
    rope = np.dot(np.squeeze(np.loadtxt("/home/joschu/Data/rope/rope%i.txt"%i)),np.diag([1,-1,-1]))    
    rope = voxel_downsample(rope,.03)
    
    #f = registration.tps_rpm
    tps = registration.tps_rpm(rope[:,:2], sim_ropes[i][:,:2],plotting=10,reg_init=1,reg_final=.025,n_iter=200)
    

def test_registration_3d():
#if __name__ == "__main__":
    import rospy, itertools, glob
    from jds_utils.colorize import colorize
    if rospy.get_name() == "/unnamed": rospy.init_node('test_registration_3d',disable_signals=True)
    data_dir = "/home/joschu/Data/rope1"
    
    files = sorted(glob.glob(osp.join(data_dir,"*.txt")))
    
    distmat1 = np.zeros((len(files), len(files)))
    distmat2 = np.zeros((len(files), len(files)))

    for (i0, i1) in itertools.combinations(xrange(12),2):
        print colorize("comparing %s to %s"%(files[i0], files[i1]),'red',bold=True)
        rope0 = np.loadtxt(osp.join(data_dir,files[i0]))
        rope1 = np.loadtxt(osp.join(data_dir,files[i1]))
        f = registration.tps_rpm(rope0, rope1, plotting=1,reg_init=100,reg_final=1,n_iter=51, verbose=False)
        ##distmat1[i0, i1] = f.cost
        #distmat2[i0, i1] = f.corr_sum
        break


    plt.figure(1)    
    plt.imshow(distmat1)
    plt.title("distances")
    plt.figure(2)
    plt.imshow(distmat2)
    plt.title("corr_sums")
    np.savez("cross_registration_results", distmat = distmat1, names = files)

#if __name__ == "__main__":
def test_cups():
    import rospy, itertools, glob
    from jds_utils.colorize import colorize
    from jds_image_proc.pcd_io import load_xyzrgb
    if rospy.get_name() == "/unnamed": rospy.init_node('test_registration_3d',disable_signals=True)
    data_dir = "/home/joschu/Data/cups"
    xyz1, rgb1 = load_xyzrgb(osp.join(data_dir,"cups1.pcd"))
    def preproc(xyz1):
        xyz1 = xyz1.reshape(-1,3)
        xyz1 = np.dot(xyz1, np.diag([1,-1,-1]))
        xyz1 = xyz1[(xyz1[:,2] > .02) & (xyz1[:,2] < .2) 
                    & (np.abs(xyz1[:,0]) < .15) & (np.abs(xyz1[:,1]) < .3)]
        xyz1 = voxel_downsample(xyz1, .015)
        return xyz1
    xyz1 = preproc(xyz1)
    
    #from mayavi import mlab
    #mlab.points3d(*xyz1.T,color=(1,1,1),scale_factor=.01)
    xyz2, rgb2 = load_xyzrgb(osp.join(data_dir,"cups4.pcd"))
    xyz2 = preproc(xyz2)
    f = registration.tps_rpm(3*xyz1, 3*xyz2, plotting=4,reg_init=1,reg_final=.05,n_iter=200, verbose=False)


def test_zrot1():
#def test_plate():
    import rospy, itertools, glob
    from jds_utils.colorize import colorize
    from jds_image_proc.pcd_io import load_xyzrgb
    import h5py
    if rospy.get_name() == "/unnamed": rospy.init_node('test_registration_3d',disable_signals=True)
    data_dir = "/home/joschu/python/lfd/data"
    f = h5py.File(osp.join(data_dir, "images/pickup-plate0.seg.h5"),"r")
    xyz1, rgb1 = np.asarray(f["plate"]["xyz"]), np.asarray(f["plate"]["rgb"])
    xyz1 = voxel_downsample(xyz1, .02)
    from numpy import sin, cos, pi
    print "true angle", pi/3
    xyz_target = xyz1.mean(axis=0)[None,:] + 1.5*np.dot(xyz1 - xyz1.mean(axis=0)[None,:], np.array([[cos(pi/3), sin(pi/3), 0], [-sin(pi/3), cos(pi/3), 0], [0, 0, 1]]))
    
    f = registration.tps_rpm_zrot(xyz1, xyz_target, plotting=1,reg_init=2,reg_final=.5,n_iter=8, verbose=False, cost_per_radian=2)

def test_zrot2():
#def test_plate():
    import rospy, itertools, glob
    from jds_utils.colorize import colorize
    from jds_utils.math_utils import normr, norms
    if rospy.get_name() == "/unnamed": rospy.init_node('test_registration_3d',disable_signals=True)
    xyz1 = np.random.randn(300,3)*.2
    xyz2 = np.random.randn(300,3)*.2
    xyz1 = xyz1[norms(xyz1,1) < 1]*.2
    xyz2 = xyz2[norms(xyz1,1) < 1]*.2
    from numpy import sin, cos, pi
    
    f = registration.tps_rpm_zrot(xyz1, xyz2, plotting=1,reg_init=2,reg_final=.5,n_iter=8, verbose=False, cost_per_radian = 2)

if __name__ == "__main__":
    test_zrot2()