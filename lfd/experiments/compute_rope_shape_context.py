from os.path import join, basename, splitext
from glob import glob
from lfd.features import cloud_shape_context
from rope_vision.clouds import voxel_downsample
import numpy as np

DATA_DIR = '/home/joschu/Data/rope'

for fname in glob(join(DATA_DIR,"*.txt")):
    if "feats" in fname: continue
    print fname
    xyz = np.loadtxt(fname)
    if "sim" in fname: xyz = np.dot(xyz, np.diag([1,-1,1]))
    else: xyz = voxel_downsample(xyz, .03)
    features = np.c_[xyz, cloud_shape_context(xyz, 2)]
    
    feat_fname = splitext(fname)[0] + ".feats.txt"
    np.savetxt(feat_fname, features)