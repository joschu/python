#!/usr/bin/env python
"""
For annotating a point cloud that corresponds to demonstration. Select which point clouds are the verb "arguments"
"""


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("infile")
parser.add_argument("outfile")
args = parser.parse_args()

assert args.outfile.endswith("npz") or args.outfile.endswith(".h5")

from point_clouds import tabletop
import matplotlib.pyplot as plt, numpy as np
import rospy
rospy.init_node("segment_point_cloud")

plt.ion()

f = np.load(args.infile)
clusters = tabletop.segment_tabletop_scene(f["xyz"],plotting2d_all=True,plotting2d=True,plotting3d=True)
ids_str = raw_input("type wanted cluster ids in order and then press enter")
object_clusters = []

for num_str in ids_str.split():
    object_clusters.append(clusters[int(num_str)])

if args.outfile.endswith("npz"):    
    np.save(args.outfile, np.array(object_clusters,dtype=object))
else:
    import h5py
    out = h5py.File(args.outfile, "w")
    for clu in object_clusters:
        out[str(clu)] = clu
    out.close()