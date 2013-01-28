from pylab import *
import scipy.spatial.distance as ssd
n = 100
x_n3 = randn(n,3)
y_n = randn(n)
x1_n4 = np.c_[x_n3, np.ones(n)]
K_nn = ssd.cdist(x_n3,x_n3)
c_4n = np.linalg.lstsq(x1_n4, K_nn)[0]
Kc_nn = K_nn - x1_n4.dot(c_4n)
_b_4 = linalg.lstsq(x1_n4, y_n)[0]
yc_n = y_n - x1_n4.dot(_b_4)
a_n = np.linalg.solve(Kc_nn, yc_n)
b_4 = _b_4 - x1_n4.T.dot(a_n)




gab = np.linalg.solve(np.r_[np.c_[K_nn, x1_n4], np.c_[x1_n4.T, np.zeros((4,4))]], np.r_[y_n, np.zeros(4)])
ga = gab[:n]
gb = gab[n:]