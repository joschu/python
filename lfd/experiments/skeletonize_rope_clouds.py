from os.path import join, basename, splitext
from glob import glob
from point_clouds.features import cloud_shape_context
from rope_vision.clouds import voxel_downsample
import numpy as np
from rope_vision.rope_initialization import find_path_through_point_cloud, get_skeleton_points

DATA_DIR = '/home/joschu/Data/rope'

for fname in glob(join(DATA_DIR,"*.txt")):
    if "feats" in fname: continue
    print fname
    xyz = np.loadtxt(fname)
    if "sim" in fname: xyz = np.dot(xyz, np.diag([1,-1,1]))
    # else: xyz = find_path_through_point_cloud(xyz)
    else: xyz = get_skeleton_points(xyz)
    
    new_fname = splitext(fname)[0] + ".skel.txt"
    np.savetxt(new_fname, xyz)