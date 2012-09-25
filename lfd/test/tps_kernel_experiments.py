import numpy as np
import scipy.spatial.distance as ssd

x = np.random.randn(10,3)
x1 = np.c_[x, np.ones((10,1))]
u,s,vh = np.linalg.svd(x1)
nullspace = u[:,4:]
nullproj = np.dot(nullspace, nullspace.T)

K = -ssd.cdist(x,x)

print np.dot(nullproj, x1)
print np.linalg.eigvalsh(np.dot(nullproj, np.dot(K,nullproj)))


           