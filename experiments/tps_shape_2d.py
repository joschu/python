from pylab import *
import scipy.spatial.distance as ssd

X = array([
    (10,0),
    (3,4),
    (4,3),
    (5,0),
    (4,-3),
    (3,-4)])

y = array([
    25,
    0,0,0,0,0])

R = ssd.cdist(X,X)
K = R**2 * log(R)
K[~np.isfinite(K)] = 0

a = lstsq(K,y)[0]
def f(Xp):
    r = ssd.cdist(Xp,X)
    k = r**2 * log(r)
    k[~np.isfinite(k)] = 0
    y = np.dot(k, a)
    return y

xts = linspace(-5, 15, 100)
yts = linspace(-8, 8, 100)


xs, ys = meshgrid(xts, yts)

D = f(c_[xs.flatten(), ys.flatten()]).reshape(xs.shape)
extents = [xts.min(), xts.max(), yts.min(), yts.max()]
imshow(D,extent = extents)
contour(D, extent=extents,levels=[0])
show()

