import tps
import numpy as np
from os.path import join

import argparse

KNOT_DATA = "/home/joschu/bulletsim/data/knots"

parser = argparse.ArgumentParser()
parser.add_argument("train_ropes")
parser.add_argument("train_poses")
parser.add_argument("test_init_rope")
parser.add_argument("test_poses")
parser.add_argument("test_warped_ropes")
args = parser.parse_args()

ropes_train = np.loadtxt(join(KNOT_DATA,args.train_ropes))
poses_train = np.loadtxt(join(KNOT_DATA,args.train_poses))


init_rope_train = ropes_train[0].reshape(-1,3)
init_rope_test = np.loadtxt(join(KNOT_DATA, args.test_init_rope))

xyz = init_rope_train
xyzp = init_rope_test

xyz = np.concatenate([init_rope_train,[[0,0,0],[0,0,1]]],0)
xyzp = np.concatenate([init_rope_test,[[0,0,0],[0,0,1]]],0)

x,y,z = xyz.T
xp,yp,zp = xyzp.T

F = tps.TPS33(x,y,z, xp, yp, zp)

quat_left = poses_train[:,0:4]
xyz_left = poses_train[:,4:7]
grip_left = poses_train[:,7]
quat_right = poses_train[:,8:12]
xyz_right = poses_train[:,12:15]
grip_right = poses_train[:,15]

pvecs_left = np.c_[xyz_left, quat_left]
pvecs_right = np.c_[xyz_right, quat_right]

warped_left = tps.transform_poses(pvecs_left, F)
warped_right = tps.transform_poses(pvecs_right, F)


poses_warped = poses_train.copy()

poses_warped[:,0:4] = warped_left[:,3:7]
poses_warped[:,4:7] = warped_left[:,0:3]
poses_warped[:,8:12] = warped_right[:,3:7]
poses_warped[:,12:15] = warped_right[:,0:3]

rope_points_orig = ropes_train.reshape(-1,3)
xs_warped, ys_warped, zs_warped = F.eval(*rope_points_orig.T).T
rope_points_warped = np.c_[xs_warped,ys_warped,zs_warped].reshape(len(poses_warped),-1)

np.savetxt(join(KNOT_DATA, args.test_poses), poses_warped, fmt="%.4f")
np.savetxt(join(KNOT_DATA, args.test_warped_ropes), rope_points_warped, fmt="%.4f")


from pylab import *
clf()

plot(poses_train[:,4], poses_train[:,5],'b-x')
plot(poses_warped[:,4], poses_warped[:,5], 'g-x')
plot(init_rope_train[:,0], init_rope_train[:,1],'b')
plot(init_rope_test[:,0], init_rope_test[:,1],'g')


Xs0,Ys0 = np.mgrid[4:8:.4, 0:4:.4]
Xs0, Ys0 = Xs0.flatten(), Ys0.flatten()
Zs0 = np.zeros(Xs0.shape)

Xs1, Ys1, Zs1 = F.eval(Xs0, Ys0, Zs0).T

mats = F.grad(Xs0, Ys0, Zs0)
for (mat,x,y,z) in zip(mats,Xs1,Ys1,Zs1):
    xax,yax,zax = mat.T
    plot([x,x+xax[0]*.1], [y,y+xax[1]*.1],'r')
    plot([x,x+yax[0]*.1], [y,y+yax[1]*.1],'g')
    
axis('equal')
    
show()
#for (x1,y1,dx1,dy1) in zip(Xs1, Ys1, dXs1, dYs1):
    #plot([x1, x1+1*dx1[0]], [y1, y1+1*dy1[0]],'r')
    #plot([x1, x1+1*dx1[1]], [y1, y1+1*dy1[1]],'g')
    
    #a = np.array([dx1, dy1])
    #q,r = qr(a)
    #ox,oy = q[:,0], q[:,1]
    #plot([x1, x1+ox[0]], [y1, y1+ox[1]],'r')
    #plot([x1, x1+oy[0]], [y1, y1+oy[1]],'b')
    


##curve1a = np.c_[Fx(curve0[:,0]+randn(100)*.01,curve0[:,1]+randn(100)*.01), Fy(curve0[:,0]+randn(100)*.01, curve0[:,1]+randn(100)*.01)]
#curve1a = np.c_[Fx(curve0[:,0],curve0[:,1]), Fy(curve0[:,0], curve0[:,1])]
#plot(curve0[:,0],curve0[:,1],'bo')
#plot(curve1[:,0],curve1[:,1],'bo')
#plot(curve1a[:,0],curve1a[:,1],'gx',ms=8,mew=3)
#show()
#plot(Xs1.flatten(),Ys1.flatten(),'r.')

#plt.axis("equal")