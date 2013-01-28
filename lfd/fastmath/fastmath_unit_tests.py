import numpy as np
from lfd import fastmath

scale_coef = 1
rot_coefs = np.array([1,2,3],float)
fastmath.set_coeffs(rot_coefs, scale_coef)


def logmap(m):
    "http://en.wikipedia.org/wiki/Axis_angle#Log_map_from_SO.283.29_to_so.283.29"
    theta = np.arccos((np.trace(m) - 1)/2)
    return (1/(2*np.sin(theta))) * np.array([m[2,1] - m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1]])

def regfunc(b):        
    if np.linalg.det(b) < 0 or np.isnan(b).any(): return np.inf
    b = b.T
    u,s,vh = np.linalg.svd(b)
    p = vh.T.dot(s.dot(vh))        
    return np.abs(np.log(s)).sum()*scale_coef + float(np.abs(logmap(u.dot(vh))).dot(rot_coefs))
x = np.random.randn(3,3) + 3*np.eye(3)
assert abs(fastmath.rot_reg(x) - regfunc(x)) < 1e-4


grad = np.empty((3,3))
for i in xrange(3):
    for j in xrange(3):
        xpert = x.copy()
        xpert[i,j] += 1e-4
        grad[i,j] = (fastmath.rot_reg(xpert) - fastmath.rot_reg(x))/1e-4
fastgrad =  fastmath.rot_reg_grad(x)
assert np.allclose(grad, fastgrad, atol=1e-3)

    
