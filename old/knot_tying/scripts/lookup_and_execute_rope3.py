import argparse


parser = argparse.ArgumentParser()
parser.add_argument("dbname")

args = parser.parse_args()

from brett2 import pr2, traj_utils, ros_utils
from knot_tying import rope_library
import numpy as np
from jds_utils.colorize import colorize
from time import sleep
import traceback
import jds_utils.transformations
import jds_utils.conversions as conv
import subprocess
from jds_utils.yes_or_no import yes_or_no

import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
import rospy
import cv2
import jds_image_proc.interactive_roi as roi
from jds_image_proc import curves

import roslib;
roslib.load_manifest("tf")

RVIZ_ROPE = 0
RVIZ_LPOS = 1
RVIZ_RPOS = 2
RVIZ_ORIG_ROPE = 3

def set_cart_control():
    cm = "rosrun pr2_controller_manager pr2_controller_manager"
    subprocess.check_call("%s stop r_arm_controller l_arm_controller"%cm,shell=True)
    subprocess.check_call("%s start r_cart l_cart"%cm,shell=True)
def set_joint_control():    
    cm = "rosrun pr2_controller_manager pr2_controller_manager"
    subprocess.check_call("%s stop r_cart l_cart"%cm,shell=True)
    subprocess.check_call("%s start r_arm_controller l_arm_controller"%cm,shell=True)

def human_get_rope():
    point_cloud = rospy.wait_for_message("/camera/depth_registered/points", sm.PointCloud2)
    
    xyz, bgr = ros_utils.pc2xyzrgb(point_cloud)
    
    xys = roi.get_polyline(bgr,'draw curve')
    uvs = np.int32(xys)[:,::-1]
    us,vs = uvs.T
    xyzs = xyz[us,vs]
    xyzs_good = xyzs[np.isfinite(xyzs).all(axis=1)]
    print "%i out of %i labeled points are good"%(len(xyzs_good), len(xyzs))
    xyzs_unif = curves.unif_resample(xyzs_good,100,tol=.002)
    return xyzs_unif
    
def transform_points(xyz, to_frame, from_frame):
    xyz = xyz.reshape(-1,3)
    xyz1 = np.c_[xyz, np.ones((xyz.shape[0],1))]
    trans, rot = brett.tf_listener.lookupTransform(to_frame, from_frame,rospy.Time(0))
    mat = conv.trans_rot_to_hmat(trans,rot)
    
    xyz1_transformed = np.dot(xyz1, mat.T)
    return xyz1_transformed[:,:3]
    
    
def quatswap(quat):
    mat = transformations.quaternion_matrix(quat)
    mat_new = np.c_[mat[:,2],mat[:,1],-mat[:,0],mat[:,3]]
    return transformations.quaternion_from_matrix(mat_new)

def normalize(v):
    return v / np.linalg.norm(v)

def execute_dual_arm_trajectory(pr2, lquat, lpos, lgrip, left_used, rquat, rpos, rgrip, right_used, rope):

    rviz.draw_curve(lpos, RVIZ_LPOS, width=.005)
    rviz.draw_curve(rpos, RVIZ_RPOS, width=.005)
    ok = yes_or_no("trajectory ok?")
    if not ok:
        print "canceling trajectory"
        return
        

    for i in xrange(len(lquat)):
        
        if left_used:
            brett.larm.set_cart_target(normalize(quatswap(lquat[i])), lpos[i], "/base_footprint")
            if lgrip[i] < .03: lgrip[i] = 0
            brett.lgrip.set_angle_target(lgrip[i])

        if right_used:
            brett.rarm.set_cart_target(normalize(quatswap(rquat[i])), rpos[i], "/base_footprint")
            if rgrip[i] < .03: rgrip[i] = 0
            brett.rgrip.set_angle_target(rgrip[i])

        rviz.draw_curve(rope[i], RVIZ_ORIG_ROPE, width=.01, rgba = (1,0,0,1))
        

        sleep(.04)

# TODO: set saturation arguments for velocity and torque
# also reduce max effort arguments

rospy.init_node("knot_tying",disable_signals=True)
library = rope_library.RopeTrajectoryLibrary(args.dbname, "write")
brett = pr2.PR2()


rviz = ros_utils.RvizWrapper()

while True:
    set_joint_control()
    brett.rarm.goto_posture("side")
    brett.larm.goto_posture("side")

    rope_k3 = transform_points(human_get_rope(), "base_footprint", "head_mount_kinect_rgb_optical_frame")

    

    
    rviz.draw_curve(rope_k3,RVIZ_ROPE,width=.02)

    _,rars = library.get_closest_and_warp(rope_k3)

    left_used = (rars["grab_l"] > -1).any()
    right_used = (rars["grab_r"] > -1).any()
    
    l_quat = rars["quat_l"]
    l_pos = rars["xyz_l"]
    r_quat = rars["quat_r"]
    r_pos = rars["xyz_r"]
        
    r_grip = rars["grip_r"]
    l_grip = rars["grip_l"]
    
    print "executing trajectory..."
    set_cart_control()
    execute_dual_arm_trajectory(pr2, l_quat, l_pos, l_grip/7, left_used, r_quat, r_pos, r_grip/7, right_used, rars["rope"])
    print "done"
    
