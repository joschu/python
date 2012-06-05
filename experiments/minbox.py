import numpy as np
from numpy import pi

xs, ys = np.mgrid[-5:5,-5:5]
xys = np.c_[xs.flat, ys.flat].T

def rot(a):
    return np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])

xys1 = np.dot(rot(pi/8), xys)


def minbox(xy_2n,n):
    # first should take convex hull
    angles = np.linspace(0,2*pi, n, endpoint = False)
    areas = np.empty(angles.size)
    for (i,angle) in enumerate(angles):
        xyrot_2n = np.dot(rot(angle), xy_2n)
        areas[i] = xyrot_2n.ptp(axis=1).prod()
        
    i_best = areas.argmin()
    angle_best = angles[i_best]
    
    xyrot_2n = np.dot(rot(angle), xy_2n)
    xmin, ymin = xyrot_2n.min(axis=1)
    xmax, ymax = xyrot_2n.max(axis=1)
    cornersrot_24 = np.array([[xmin, xmin, xmax, xmax],
                              [ymin, ymax, ymin, ymax]])
    return np.dot(rot(-angle_best),cornersrot_24)
    
print np.dot(rot(-pi/8),minbox(xys1, 16))
                             