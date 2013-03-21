import numpy as np
import os.path as osp
import os
from glob import glob
import lfd.tps as tps
from jds_utils.colorize import colorize
from lfd import fastmath, registration
import iros.image_registration as ir


PLOT = False

def axisanglepart(m):
    if np.linalg.det(m) < 0 or np.isnan(m).any(): raise Exception
    u,s,vh = np.linalg.svd(m)
    rotpart = u.dot(vh)
    theta = np.arccos((np.trace(rotpart) - 1)/2)   
    axis =  (1/(2*np.sin(theta))) * np.array([[rotpart[2,1] - rotpart[1,2], rotpart[0,2]-rotpart[2,0], rotpart[1,0]-rotpart[0,1]]])
    return theta, axis


    
def fit(demo_keypts, current_keypts):    
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
    f = registration.ThinPlateSpline()
    f.lin_ag = lin_ag
    f.trans_g = trans_g
    f.w_ng = w_ng
    f.x_na = demo_keypts
    
    return f
    
def xyz2rc(xyz, xyz_img):
    diffs_rc = ((xyz_img - xyz)**2).sum(axis=2)
    diffs_rc[np.isnan(diffs_rc)] = 1e100
    mindist = diffs_rc.min()
    if mindist > 1e-4: 
        print "warning: nonzero minimum distance:",mindist
    return np.unravel_index(diffs_rc.argmin(), diffs_rc.shape)    
    
def test_fitting():
    testdata_dir = osp.join(os.getenv("IROS_DATA_DIR"), "testdata")
    fnames = glob(osp.join(testdata_dir,"segment*.npz"))
    np.set_printoptions(precision=3)    

    for (i,fname) in enumerate(fnames):
        f = np.load(fname)
        (current_xyz, current_rgb, current_keypts,demo_xyz, demo_rgb, demo_keypts) = \
            [f[key] for key in ["current_xyz", "current_rgb", "exec_keypts", "demo_xyz", "demo_rgb", "demo_keypts"]]
        f = fit(demo_keypts, current_keypts)

def test_keypoint_matching():
    iros_data_dir = osp.join(os.getenv("IROS_DATA_DIR"))
    testdata_dir = osp.join(os.getenv("IROS_DATA_DIR"), "testdata")
    fnames = glob(osp.join(testdata_dir,"segment*.npz"))
    np.set_printoptions(precision=3)    
    for (i,fname) in enumerate(fnames):

        print "**** dataset %i ****"%i
        f = np.load(fname)
        (current_xyz, current_rgb, current_keypts,demo_xyz, demo_rgb, demo_keypts) = \
            [f[key] for key in ["current_xyz", "current_rgb", "exec_keypts", "demo_xyz", "demo_rgb", "demo_keypts"]]
        
        seg_num = int(osp.basename(fname)[7:9])
        demo_rgb = np.load(glob(osp.join(iros_data_dir, "InterruptedSuture0", 'point_clouds', 'pt%i/seg%i_*_rgb_*.npy'%(0, seg_num)))[0])
        current_xy = np.array([xyz2rc(xyz, current_xyz) for xyz in current_keypts])[:,::-1]
        demo_xy = np.array([xyz2rc(xyz, demo_xyz) for xyz in demo_keypts])[:,::-1]
        pred_current_xy = ir.register_images(demo_rgb, current_rgb, demo_xy)
        for i in xrange(len(demo_xy)):            
            print "(%i, %i), (%i, %i), (%i, %i)"%tuple(current_xy[i].tolist() + pred_current_xy[i].tolist() + (current_xy[i] - pred_current_xy[i]).tolist())
        cur_plot_img = current_rgb.copy()
        demo_plot_img = demo_rgb.copy()
        plot_points_numbered(cur_plot_img, pred_current_xy, ir.Colors.GREEN)
        plot_points_numbered(cur_plot_img, current_xy, ir.Colors.BLUE)
        plot_points_numbered(demo_plot_img, demo_xy, ir.Colors.RED)
        while cv2.waitKey(10)==-1:
            cv2.imshow("current_rgb", cur_plot_img)
            cv2.imshow("demo_rgb", demo_plot_img)


def plot_points_numbered(plot_img, xys, color=(255,255,255)):
    for (i, (col, row)) in enumerate(xys):            
        cv2.putText(plot_img, str(i), (col, row), cv2.FONT_HERSHEY_PLAIN, 1.0, color, thickness = 1)

if __name__ == "__main__":
    import cv2
    test_keypoint_matching()