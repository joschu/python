import scipy.optimize as so
import lfd.registration as lr
import lfd.warping as lw
import numpy as np
import rospy
import time
if rospy.get_name() == "/unnamed":
    rospy.init_node("test_rigidity", disable_signals=True)



pts0 = np.loadtxt("rope_control_points.txt")
pts1 = np.loadtxt("bad_rope.txt")




tps = lr.ThinPlateSpline.identity(3)
tps.fit(pts0, pts1, smoothing=.01, angular_spring = 10)

lr.plot_orig_and_warped_clouds(tps, pts0, pts1, res=.04)                



_, rots = tps.transform_frames(pts0, np.eye(3)[None,:,:], orthogonalize=False)
evs = np.array([np.linalg.eigvals(rot) for rot in rots])
print np.abs(evs).min(axis=1).min()
print np.min([np.linalg.det(rot) for rot in rots])

tps = lr.tps_rpm(pts0, pts1, n_iter=200, plotting=5, verbose=False, rad_final = .001, reg_init=1)


_, rots = tps.transform_frames(pts0, np.eye(3)[None,:,:], orthogonalize=False)
evs = np.array([np.linalg.eigvals(rot) for rot in rots])
print np.abs(evs).min(axis=1).min()
print np.min([np.linalg.det(rot) for rot in rots])


#lw.draw_grid(rviz, tps, pts0.min(axis=0), pts1.min(axis=0), 'base_footprint')
#dist_mn = ssd.cdist(x_md, self.x_nd)
#grad_mdd = np.zeros((m,d,d))
#for i_d in xrange(d):
    #diffs_mn = x_md[:,i_d][:,None] - self.x_nd[:,i_d][None,:]
    #if self.d == 2:
        #raise NotImplementedError
    #elif self.d == 3:
        #grad_mdd[:,:,i_d] = self.a_Dd[i_d,:][None,:] + np.dot(nan2zero(diffs_mn / dist_mn),self.w_nd[:,i_d])[:,None]
#newrot_mdd = (grad_mdd[:,:,:,None]* rot_mdd[:,None,:,:]).sum(axis=2)
