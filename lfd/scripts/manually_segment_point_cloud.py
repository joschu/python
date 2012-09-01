#!/usr/bin/env python
"""
For annotating a point cloud that corresponds to demonstration. Select which point clouds are the verb "arguments"
"""


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("infile")
parser.add_argument("--outfile")
args = parser.parse_args()

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
import os
import h5py
from snazzy_msgs.srv import *

rospy.init_node("manually_segment_point_cloud")
listener = tf.TransformListener()
seg_svc = rospy.ServiceProxy("interactive_segmentation", ProcessCloud)

if args.infile.endswith("npz"):
    f = np.load(args.infile)
    xyz0, rgb0 = f["xyz"], f["bgr"]
    pc = ros_utils.xyzrgb2pc(xyz0, rgb0, "base_footprint")
    
else:
    raise NotImplementedError

object_names = raw_input("type object names separated by spaces\n").split()
outfilename  = os.path.splitext(args.infile)[0] + ".seg.h5"
if os.path.exists(outfilename): os.remove(outfilename)
outfile = h5py.File(outfilename, "w")

for object_name in object_names:
    pc_sel = seg_svc.call(ProcessCloudRequest(cloud_in = pc)).cloud_out
    xyz, rgb = ros_utils.pc2xyzrgb(pc_sel)
    outfile.create_group(object_name)
    outfile[object_name]["xyz"] = xyz
    outfile[object_name]["rgb"] = rgb
    