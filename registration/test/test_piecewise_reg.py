from registration import piecewise_registration as pr
import registration
from pylab import *
import numpy as np
import os.path as osp
data_dir = osp.join(osp.dirname(registration.__file__),"data")


# src0, src1 = np.load(osp.join(data_dir, "clouds0.npy"))
src0 = randn(100,3) * np.array([[2,1,1]]) + np.array([[10,0,0]])
src1 = randn(100,3) + np.array([[10,10,0]])


targ0 = randn(100,3) * np.array([[2,1,1]]) + np.array([[30,0,0]])
targ1 = randn(100,3) + np.array([[-10,0,0]])



src = np.r_[src0, src1]
# targ0, targ1 = np.load(osp.join(data_dir, "clouds1.npy"))
targ = np.r_[targ0, targ1]

f = pr.InterpolatedTransformation()
f.fit([src0, src1], [targ0, targ1], transform_type = "rigid_planar")

from lfd.warping import draw_grid
# draw_grid(rviz, lambda x: f.transform_points(x[:,:2]), src.min(axis=0), src.max(axis=0),
#           "base_footprint", xres = .03, yres = .03, zres = .02)
from lfd.registration import plot_warped_grid_2d

plot(src0[:,1], src0[:,0], 'r.')
plot(src1[:,1], src1[:,0], 'r.')
plot(targ0[:,1], targ0[:,0], 'b.')
plot(targ1[:,1], targ1[:,0], 'b.')


msrc = f.transform_points(src)
plot(msrc[:,1], msrc[:,0],'g.')


f2d = lambda x: f.transform_points(np.c_[x, np.zeros((len(x),1))])[:,:2]
plot_warped_grid_2d(f2d, src.min(axis=0)[:2], src.max(axis=0)[:2], grid_res = .5)
axis('equal')
show()