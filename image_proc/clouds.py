import numpy as np
from collections import defaultdict


def voxel_downsample(xyz,s):
    xyz = xyz.reshape(-1,3)
    d = defaultdict(list)
    for (i,pt) in enumerate(xyz):
        x,y,z = pt
        d[(int(x//s),int(y//s),int(z//s))].append(i)
    return np.array([xyz[inds].mean(axis=0) for inds in d.values()])

