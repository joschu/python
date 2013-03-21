import numpy as np
import os.path as osp
import os
from glob import glob
import lfd.tps as tps
from jds_utils.colorize import colorize
from lfd import fastmath
testdata_dir = osp.join(os.getenv("IROS_DATA_DIR"), "testdata")
fnames = glob(osp.join(testdata_dir,"segment*.npz"))
np.set_printoptions(precision=3)

PLOT = False

def axisanglepart(m):
    if np.linalg.det(m) < 0 or np.isnan(m).any(): raise Exception
    u,s,vh = np.linalg.svd(m)
    rotpart = u.dot(vh)
    theta = np.arccos((np.trace(rotpart) - 1)/2)   
    axis =  (1/(2*np.sin(theta))) * np.array([[rotpart[2,1] - rotpart[1,2], rotpart[0,2]-rotpart[2,0], rotpart[1,0]-rotpart[0,1]]])
    return theta, axis


for (i,fname) in enumerate(fnames):
    f = np.load(fname)
    (current_xyz, current_rgb, current_keypts,demo_xyz, demo_rgb, demo_keypts) = \
        [f[key] for key in ["current_xyz", "current_rgb", "exec_keypts", "demo_xyz", "demo_rgb", "demo_keypts"]]
    
    
    
    rot_coefs = .01*np.array((0.01,0.01,0.0025))
    scale_coef= .01*.01
    fastmath.set_coeffs(rot_coefs, scale_coef)
    rfunc = fastmath.rot_reg
    lin_ag, trans_g, w_ng = tps.tps_fit_regrot(demo_keypts, current_keypts, bend_coef = 1, rfunc=rfunc)
    #spline.fit(demo_keypts, current_keypts,bend_coef=10,rot_coef=.1)
    print "-----------------------------"
    print "dataset %i"%i
    centertrans_g = tps.tps_eval([demo_keypts.mean(axis=0)], lin_ag, trans_g, w_ng, demo_keypts)[0]-demo_keypts.mean(axis=0)
    print "translation of center:\n", centertrans_g
    print "linear:\n",lin_ag
    print "axis-angle", axisanglepart(lin_ag.T)    
    print "singular vals", np.linalg.svd(lin_ag.T)[1]
    print "nonlinear:\n",w_ng
    print "residuals:\n",tps.tps_eval(demo_keypts, lin_ag, trans_g, w_ng, demo_keypts)-current_keypts
    max_residual = (tps.tps_eval(demo_keypts, lin_ag, trans_g, w_ng, demo_keypts)-current_keypts).max()
    print "max residual:%.3f"%max_residual
    if max_residual > .015:
        print colorize("warning: large residual (%.3f)"%max_residual,'red',highlight=True)
    if np.linalg.norm(centertrans_g) > .4:
        print colorize("warning: large translation (%.3f)"%np.linalg.norm(centertrans_g),'red',highlight=True)
        
    if PLOT:
        import mayavi.mlab as mlab
        mlab.clf()
        x,y,z = demo_keypts.T
        mlab.points3d(x,y,z,color=(0,1,0))
        x,y,z = current_keypts.T
        mlab.points3d(x,y,z,color=(1,0,0))
        for (demopt, curpt) in zip(demo_keypts, current_keypts):
            mlab.plot3d([demopt[0],curpt[0]], [demopt[1],curpt[1]], [demopt[2], curpt[2]],color=(1,1,0),tube_radius=.001)
        mlab.show()
