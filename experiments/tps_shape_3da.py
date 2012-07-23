from __future__ import division
from pylab import *
import scipy.spatial.distance as ssd
from image_proc.clouds import voxel_downsample
from mayavi import mlab
from image_proc.pcd_io import load_xyzrgb
from image_proc.pcl_utils import xyz2uv, uv2xyz
import scipy.ndimage as ndi
xyz_orig, _ = load_xyzrgb("/home/joschu/Proj/shape_comp/cup.pcd")

us, vs = mgrid[:640, :480]

umin, umax = 220,330
vmin, vmax = 230,290
xres = yres = zres = .002

xyz = xyz_orig[umin:umax, vmin:vmax, :]
us = us[umin:umax, vmin:vmax]
vs = vs[umin:umax, vmin:vmax]
good_pts = xyz[np.isfinite(xyz[:,:,2])]


def laplacian(F):
    Fxx = F[2:,1:-1,1:-1] + F[:-2,1:-1,1:-1] - 2*F[1:-1,1:-1,1:-1]
    Fyy = F[1:-1,2:,1:-1] + F[1:-1,:-2,1:-1] - 2*F[1:-1,1:-1,1:-1]
    Fzz = F[1:-1,1:-1,2:] + F[1:-1,1:-1,:-2] - 2*F[1:-1,1:-1,1:-1]
    return Fxx + Fyy + Fzz

def curv_grad(F):
    return laplacian(laplacian(F))
def curv_cost(F):

    Fxx = F[2:,:,:] + F[:-2,:,:] - 2*F[1:-1,:,:]
    Fyy = F[:,2:,:] + F[:,:-2,:] - 2*F[:,1:-1,:]
    Fzz = F[:,:,2:] + F[:,:,:-2] - 2*F[:,:,1:-1]

    Fxy = F[1:,1:,:] + F[:-1,:-1,:] - F[1:,:-1,:] - F[:-1,1:,:]
    Fyz = F[:,1:,1:] + F[:,:-1,:-1] - F[:,1:,:-1] - F[:,:-1,1:]
    Fxz = F[1:,:,1:] + F[:-1,:,:-1] - F[1:,:,:-1] - F[:-1,:,1:]

    return (Fxx**2).sum()+(Fyy**2).sum()+(Fzz**2).sum()+\
           2*(Fxy**2).sum()+2*(Fyz**2).sum()+2*(Fxz**2).sum()

    
def xyz2uv(xyz): 
 # http://www.pcl-users.org/Using-Kinect-with-PCL-How-to-project-a-3D-point-x-y-z-to-the-depth-rgb-image-and-how-to-unproject-a--td3164499.html
    
    
    CX = 320
    CY = 240
    F = 525   
    
    xyz = np.atleast_2d(xyz)
    x,y,z = xyz.reshape(-1,3).T
    
  
    v = F*(x / z) + CX;
    u = F*(y / z) + CY;
    uvs = np.c_[u,v]
    if xyz.ndim == 3:
      return uvs.reshape(xyz.shape[0], xyz.shape[1], 2)
    else:
      return uvs.reshape(xyz.shape[0],2)    
    
    
xmin, ymin, zmin = good_pts.reshape(-1,3).min(axis=0)
xmax, ymax, zmax = good_pts.reshape(-1,3).max(axis=0)


Xs, Ys, Zs = np.mgrid[xmin:xmax:xres, ymin:ymax:yres, zmin:zmax:zres]

UVs = np.round(xyz2uv(np.c_[Xs.flatten(), Ys.flatten(), Zs.flatten()]).reshape(Xs.shape+(2,))).astype('int')
Us = UVs[:,:,:,0]
Vs = UVs[:,:,:,1]
                

closer_mask = xyz_orig[Us,Vs,2] >= Zs
far_mask = xyz_orig[Us,Vs,2] < Zs
nan_mask = np.isnan(xyz_orig[Us,Vs,2])


vis_pts = np.c_[Xs[closer_mask], Ys[closer_mask], Zs[closer_mask]]
hid_pts = np.c_[Xs[far_mask], Ys[far_mask], Zs[far_mask]]
nan_pts = np.c_[Xs[nan_mask], Ys[nan_mask], Zs[nan_mask]]



OCC = np.zeros(Xs.shape, bool)
ijk = (xyz - np.array([[[xmin, ymin, zmin]]]))/np.array([[[xres, yres, zres]]])
ijk[np.isnan(ijk)] = -1
ijk = np.floor(np.clip(ijk, 0, np.array([[Xs.shape]])-1)).astype('int')




for (i,j,k) in ijk.reshape(-1,3):
    if i>0:
        OCC[i,j,k] = 1

occ_pts = np.c_[Xs[OCC], Ys[OCC], Zs[OCC]]

    
D = ndi.distance_transform_edt(1-OCC).astype('float')
D[far_mask] = -1    

last_cost = inf
step_size = 1e-4
for i in xrange(20000):

    Dmaybe = D.copy()
    Dmaybe[2:-2,2:-2,2:-2] -= curv_grad(D)*step_size
    np.putmask(Dmaybe, OCC, fmax(Dmaybe,-.1))
    np.putmask(Dmaybe, OCC, fmin(Dmaybe, .1))
    #np.putmask(Dmaybe, OCC, 0)
    np.putmask(Dmaybe, closer_mask, fmax(Dmaybe, 0))

    cost = curv_cost(Dmaybe)
    
    if cost < last_cost:
        step_size *= 1.1        
        D = Dmaybe
        print i,"hooray, decreased cost. step size: %.3e"%step_size
    else:
        step_size *= .5
        print i,"failed to decrease cost. step size: %.3e"%step_size
    if i%100 == 0: print i,step_size
    
    last_cost = cost    

    if step_size < 1e-10: break
    


mlab.clf()
#mlab.points3d(nan_pts[:,0], nan_pts[:,1], nan_pts[:,2],scale_factor=.001, color=(1,0,0))
#mlab.points3d(hid_pts[:,0], hid_pts[:,1], hid_pts[:,2],scale_factor=.001, color=(0,0,1))
#mlab.points3d(occ_pts[:,0], occ_pts[:,1], occ_pts[:,2],scale_factor=.0016, color=(0,1,0))


mlab.points3d([0],[0],[0], scale_factor=.01, color=(0,1,0))

#mlab.points3d(good_pts[:,0], good_pts[:,1], good_pts[:,2], scale_factor=.001, color=(1,1,0))

mlab.contour3d(Xs,Ys,Zs,D,opacity=.5,contours=[0])



#Zs[

#ctrl_pts = []

#for pt in good_pts:
    #for s in [1,.95,.9]:
        #ctrl_pts.append(s * pt)

##ctrl_pts.extend(good_pts)
#ctrl_pts.append([0,0,0])
#ctrl_pts = np.array(ctrl_pts)

#def voxel_downsample(xyz,s, return_inds = False):
    #xyz = xyz.reshape(-1,3)
    #xyz = xyz[np.isfinite(xyz[:,0])]
    #pts = []
    #keys = set([])
    #for (i,pt) in enumerate(xyz):
        #x,y,z = pt
        #key = (int(x//s),int(y//s),int(z//s))
        #if key not in keys:
            #keys.add(key)
            #pts.append(pt)
        
    #return np.array(pts)

#X = voxel_downsample(ctrl_pts, .02)

#y = dists = ssd.cdist(X, good_pts).min(axis=1)**2




##X = array([
    ##(10,0),
    ##(3,4),
    ##(4,3),
    ##(5,0),
    ##(4,-3),
    ##(3,-4)])

##y = array([
    ##25,
    ##0,0,0,0,0])

#R = ssd.cdist(X,X)
##K = R**2 * log(R)
#K = R
#K[~np.isfinite(K)] = 0

#a = lstsq(K+0*np.eye(K.shape[0]),y)[0]
#def f(Xp):
    #r = ssd.cdist(Xp,X)
    #k = r
    #k[~np.isfinite(k)] = 0
    #y = np.dot(k, a)
    #return y

##xts = linspace(-5, 15, 100)
##yts = linspace(-8, 8, 100)


##xs, ys = meshgrid(xts, yts)

##D = f(c_[xs.flatten(), ys.flatten()]).reshape(xs.shape)
##extents = [xts.min(), xts.max(), yts.min(), yts.max()]
##imshow(D,extent = extents)
##contour(D, extent=extents,levels=[0])
##show

#xmin, ymin, zmin = X.min(axis=0)
#xmax, ymax, zmax = X.max(axis=0)

#xts, yts, zts = np.mgrid[xmin:xmax:100j, ymin:ymax:100j, zmin:zmax:100j]
#Xp = c_[xts.flatten(), yts.flatten(), zts.flatten()]
#scalars = f(Xp).reshape(xts.shape)

#mlab.clf()
#mlab.contour3d(xts, yts, zts, scalars,contours=[0],opacity=.4)
##mlab.points3d(*good_pts.T,scale_factor=.01)
#mlab.points3d(good_pts[::4,0], good_pts[::4,1], good_pts[::4,2],scale_factor=.0025, color=(1,1,0))
#mlab.points3d(X[:,0],X[:,1], X[:,2],scale_factor=.0035, color=(1,0,0),opacity=.5)