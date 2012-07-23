from pylab import *
import scipy.spatial.distance as ssd
from image_proc.clouds import voxel_downsample
from mayavi import mlab
from image_proc.pcd_io import load_xyzrgb

xyz, bgr = load_xyzrgb("/home/joschu/Proj/shape_comp/cup.pcd")

xyz = xyz[200:330, 200:330, :]
good_pts = xyz[np.isfinite(xyz[:,:,2])]



ctrl_pts = []

for pt in good_pts:
    for s in [1,.95,.9]:
        ctrl_pts.append(s * pt)

#ctrl_pts.extend(good_pts)
ctrl_pts.append([0,0,0])
ctrl_pts = np.array(ctrl_pts)

def voxel_downsample(xyz,s, return_inds = False):
    xyz = xyz.reshape(-1,3)
    xyz = xyz[np.isfinite(xyz[:,0])]
    pts = []
    keys = set([])
    for (i,pt) in enumerate(xyz):
        x,y,z = pt
        key = (int(x//s),int(y//s),int(z//s))
        if key not in keys:
            keys.add(key)
            pts.append(pt)
        
    return np.array(pts)

X = voxel_downsample(ctrl_pts, .02)

y = dists = ssd.cdist(X, good_pts).min(axis=1)**2




#X = array([
    #(10,0),
    #(3,4),
    #(4,3),
    #(5,0),
    #(4,-3),
    #(3,-4)])

#y = array([
    #25,
    #0,0,0,0,0])

R = ssd.cdist(X,X)
#K = R**2 * log(R)
K = R
K[~np.isfinite(K)] = 0

a = lstsq(K+0*np.eye(K.shape[0]),y)[0]
def f(Xp):
    r = ssd.cdist(Xp,X)
    k = r
    k[~np.isfinite(k)] = 0
    y = np.dot(k, a)
    return y

#xts = linspace(-5, 15, 100)
#yts = linspace(-8, 8, 100)


#xs, ys = meshgrid(xts, yts)

#D = f(c_[xs.flatten(), ys.flatten()]).reshape(xs.shape)
#extents = [xts.min(), xts.max(), yts.min(), yts.max()]
#imshow(D,extent = extents)
#contour(D, extent=extents,levels=[0])
#show

xmin, ymin, zmin = X.min(axis=0)
xmax, ymax, zmax = X.max(axis=0)

xts, yts, zts = np.mgrid[xmin:xmax:100j, ymin:ymax:100j, zmin:zmax:100j]
Xp = c_[xts.flatten(), yts.flatten(), zts.flatten()]
scalars = f(Xp).reshape(xts.shape)

mlab.clf()
mlab.contour3d(xts, yts, zts, scalars,contours=[0],opacity=.4)
#mlab.points3d(*good_pts.T,scale_factor=.01)
mlab.points3d(good_pts[::4,0], good_pts[::4,1], good_pts[::4,2],scale_factor=.0025, color=(1,1,0))
mlab.points3d(X[:,0],X[:,1], X[:,2],scale_factor=.0035, color=(1,0,0),opacity=.5)