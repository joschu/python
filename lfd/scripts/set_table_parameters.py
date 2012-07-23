#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--downshift",type=float, default=.006)
args = parser.parse_args()

from brett2.PR2 import PR2, ros_utils
import rospy
from point_clouds import tabletop
import sensor_msgs.msg as sm
rospy.init_node("set_table_params")
pr2 = PR2()
pr2.rarm.goto_posture('side')
pr2.larm.goto_posture('side')
pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)
pc_tf = ros_utils.transformPointCloud2(pc, pr2.tf_listener, "base_footprint", pc.header.frame_id)
xyz,rgb = ros_utils.pc2xyzrgb(pc_tf)
table_bounds = list(tabletop.get_table_dimensions(xyz))
table_bounds[-1] -= args.downshift
table_bounds[-2] -= args.downshift
print "table bounds", table_bounds
rospy.set_param("table_bounds", " ".join([str(x) for x in table_bounds]))
