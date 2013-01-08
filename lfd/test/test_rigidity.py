import scipy.optimize as so
import lfd.registration as lr
import lfd.warping as lw
import numpy as np
import rospy
import time
import scipy.spatial.distance as ssd
import numdifftools as ndt
from jds_image_proc.clouds import voxel_downsample
from brett2.ros_utils import RvizWrapper
from jds_utils.colorize import colorize

if 1:
    if rospy.get_name() == "/unnamed":
        rospy.init_node("test_rigidity", disable_signals=True)
        from time import sleep
        sleep(1)
    rviz = RvizWrapper.create()

np.random.seed(1)

if 1:
    pts0 = np.loadtxt("rope_control_points.txt")
    pts1 = np.loadtxt("bad_rope.txt")

    
    pts_rigid = voxel_downsample(pts0, .02)
    #pts_rigid += np.random.randn(*pts_rigid.shape)
    gN = len(pts0)
    gM = len(pts_rigid)

#pts0 = pts0 + np.random.randn(*pts0.shape)*.1
#pts1 = pts1 + np.random.randn(*pts1.shape)*.1
if 0:
    gN = 100
    gM = 20
    
    pts0 = np.random.randn(gN,3)
    pts1 = np.random.randn(gN,3)

from lfd.tps import *    
    
    
if 0:
    D = 3
    def err_partial(params):
        lin_ag = params[0:9].reshape(3,3)
        trans_g = params[9:12]
        w_ng = params[12:].reshape(-1,3)
        return tps_nonrigidity(pts_eval, lin_ag, trans_g, w_ng, pts0)    
    lin_ag, trans_g, w_ng = np.random.randn(D,D), np.random.randn(D), np.random.randn(gN, D)
    J1 = tps_nonrigidity_grad(pts_eval, lin_ag, trans_g, w_ng, pts0)
    J = ndt.Jacobian(err_partial)(np.r_[lin_ag.flatten(), trans_g.flatten(), w_ng.flatten()]).reshape(-1,3,3)
    assert np.allclose(J1, J)
if 0:
    D = 3
    lin_ag, trans_g, w_ng, x_na = np.random.randn(D,D), np.random.randn(D), np.random.randn(gN, D), pts0    
    def eval_partial(x_ma_flat):
        x_ma = x_ma_flat.reshape(-1,3)
        return tps_eval(x_ma, lin_ag, trans_g, w_ng, pts0)
    for i in xrange(len(pts_eval)):
        rots = ndt.Jacobian(eval_partial)(pts_eval[i])
        rots1 = tps_grad(pts_eval[i:i+1], lin_ag, trans_g, w_ng, pts0)
        assert np.allclose(rots1, rots)

if 0:
    lin_ag, trans_g, w_ng = tps_fit(pts0, pts1, .01, 0)    
    lin2_ag, trans2_g, w2_ng = tps_fit2(pts0, pts1, .01, 0)
    
    assert np.allclose(lin_ag, lin2_ag)
    assert np.allclose(trans_g, trans2_g)
    assert np.allclose(w_ng, w2_ng)

if 1:
    lr.Globals.setup()
    np.seterr(all='ignore')
    np.set_printoptions(suppress=True)

    pts_rigid = pts0
    lin_ag, trans_g, w_ng = tps_nr_fit_enhanced(pts0, pts1, 0.01, pts0, .1, method="newton",plotting=4)
    #lin_ag2, trans_g2, w_ng2 = tps_fit(pts0, pts1, .01, .01)
    #assert np.allclose(w_ng, w_ng2)
    def eval_partial(x_ma):
        return tps_eval(x_ma, lin_ag, trans_g, w_ng, pts0) 
    lr.plot_orig_and_warped_clouds(eval_partial, pts0, pts1, res=.008)
    #handles = lw.draw_grid(rviz, eval_partial, pts0.min(axis=0), pts0.max(axis=0), 'base_footprint')

    grads = tps_grad(pts_rigid, lin_ag, trans_g, w_ng, pts0)
    print "worst violation:",np.max(np.abs([grad.T.dot(grad)-np.eye(3) for grad in grads]))
    #for grad in grads:
        #print 
# L0, T0, W0 = tps_fit(pts0, pts1, 0)
# L1, T1, W1 = tps_fit2(pts0, pts1)

    #J1q = J1.reshape(gM, 3, 3, gN,3)
    #Jq = J.reshape(gM, 3, 3, gN, 3)


#J1 = nr_grads(pts_eval, tps.a_Dd, tps.w_nd, tps.x_nd)
#_, rots = tps.transform_frames(pts0, np.eye(3)[None,:,:], orthogonalize=False)
#evs = np.array([np.linalg.eigvals(rot) for rot in rots])
#print np.abs(evs).min(axis=1).min()
#print np.min([np.linalg.det(rot) for rot in rots])

#tps = lr.tps_rpm(pts0, pts1, n_iter=200, plotting=5, verbose=False, rad_final = .001, reg_init=1)




