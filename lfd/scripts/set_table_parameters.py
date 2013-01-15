#!/usr/bin/env python
import argparse
from brett2.PR2 import PR2, ros_utils
import rospy
from point_clouds import tabletop
import sensor_msgs.msg as sm

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--downshift",type=float, default=0)
    parser.add_argument("--input_cloud",type=str, default="/camera/depth_registered/points")
    parser.add_argument("--manual_val", type=str, default="")
    args = parser.parse_args()
    return args

def move_to_init_pos(pr2, head_angle=1.1):
    pr2.head.set_pan_tilt(0, head_angle)
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')

def do_set_table_params(args):
    if args.manual_val:
        rospy.set_param("table_bounds", args.manual_val)
    else:
        pc = rospy.wait_for_message(args.input_cloud, sm.PointCloud2)
        pc_tf = ros_utils.transformPointCloud2(pc, pr2.tf_listener, "base_footprint", pc.header.frame_id)
        xyz,rgb = ros_utils.pc2xyzrgb(pc_tf)
        table_bounds = list(tabletop.get_table_dimensions(xyz))
        rospy.set_param("table_height", float(table_bounds[-1]))
        table_bounds[-1] -= args.downshift
        table_bounds[-2] -= args.downshift
        print "table bounds", table_bounds
        rospy.set_param("table_bounds", " ".join([str(x) for x in table_bounds]))

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed": 
        rospy.init_node("set_table_params", disable_signals=True)

    args = get_args()

    pr2 = PR2()
    move_to_init_pos(pr2)

    do_set_table_params(args)
