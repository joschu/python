#!/usr/bin/env python

"""
Get one point cloud and transform it into base_footprint, and save it in numpy format
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("out")
parser.add_argument("--cloud_in",default="/drop/points",nargs = "?")
args = parser.parse_args()

from brett2 import ros_utils
from utils import conversions
import roslib
roslib.load_manifest('tf')
import tf
import rospy
import numpy as np
import sensor_msgs.msg as sm
from brett2.ros_utils import transformPointCloud2

rospy.init_node("get_point_cloud")
listener = tf.TransformListener()
rospy.sleep(.3)
pc = rospy.wait_for_message(args.cloud_in, sm.PointCloud2)
pc_tf = transformPointCloud2(pc, listener, "base_footprint", pc.header.frame_id)
xyz, bgr = ros_utils.pc2xyzrgb(pc_tf)
np.savez(args.out, xyz=xyz, bgr=bgr)
