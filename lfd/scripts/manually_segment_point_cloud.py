#!/usr/bin/env python
"""
For annotating a point cloud that corresponds to demonstration. Select which point clouds are the verb "arguments"
"""


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("infile")
parser.add_argument("outfile")
args = parser.parse_args()

assert args.outfile.endswith("npy") or args.outfile.endswith(".h5")

from point_clouds import tabletop
import numpy as np
import rospy
import roslib
roslib.load_manifest('tf')
roslib.load_manifest('snazzy_msgs')
import tf
from brett2.ros_utils import pc2xyzrgb
import sensor_msgs.msg as sm
from brett2 import ros_utils
from snazzy_msgs.srv import *

rospy.init_node("manually_segment_point_cloud")
listener = tf.TransformListener()
seg_svc = rospy.ServiceProxy("interactive_segmentation", ProcessCloud)

f = np.load(args.infile)
n_obj = int(raw_input("number of objects? "))
object_clusters = []


pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)
pc_tf = ros_utils.transformPointCloud2(pc, listener, "base_footprint", pc.header.frame_id)

for _ in xrange(n_obj):
    pc_sel = seg_svc.call(ProcessCloudRequest(cloud_in = pc_tf)).cloud_out
    xyz, rgb = ros_utils.pc2xyzrgb(pc_sel)
    object_clusters.append(xyz.reshape(-1,3))
    



if args.outfile.endswith("npy"):    
    np.save(args.outfile, np.array(object_clusters,dtype=object))
else:
    import h5py
    out = h5py.File(args.outfile, "w")
    for clu in object_clusters:
        out[str(clu)] = clu
    out.close()