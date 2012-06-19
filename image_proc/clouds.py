import numpy as np
from collections import defaultdict


def voxel_downsample(xyz,s):
    xyz = xyz.reshape(-1,3)
    xyz = xyz[np.isfinite(xyz[:,0])]
    d = defaultdict(list)
    for (i,pt) in enumerate(xyz):
        x,y,z = pt
        d[(int(x//s),int(y//s),int(z//s))].append(i)
    return np.array([xyz[inds].mean(axis=0) for inds in d.values()])

def pixel_downsample(xy, s):
    xy = xy[np.isfinite(xy[:,0])]
    d = defaultdict(list)
    for (i,pt) in enumerate(xy):
        x,y = pt
        d[(int(x//s), int(y//s))].append(i)
    return np.array([xyz[inds].mean(axis=0) for inds in d.values()])