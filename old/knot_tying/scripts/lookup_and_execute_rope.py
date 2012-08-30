import argparse


parser = argparse.ArgumentParser()
parser.add_argument("dbname")

args = parser.parse_args()

from brett2 import pr2, traj_utils
from comm import comm
from knot_tying import rope_library
import numpy as np
from jds_utils.colorize import colorize
from time import sleep
import traceback
import jds_utils.transformations
import brett2.trajectories as traj


import roslib
roslib.load_manifest("brett_utils")
import rospy
from brett_utils import rviz_utils as rviz
import geometry_msgs.msg as gmm
roslib.load_manifest("pr2_python")
from pr2_python.controller_manager_client import ControllerManagerClient

def draw_curve(arr):
    pose_array = gmm.PoseArray()
    for (x,y,z) in arr:
        pose = gmm.Pose()
        pose.position = gmm.Point(x,y,z)
        pose_array.poses.append(pose)
        pose_array.header.frame_id = "/base_footprint"
        pose_array.header.stamp = rospy.Time.now()
    rviz.draw_curve(pose_array)
    
def quatswap(quat):
    mat = transformations.quaternion_matrix(quat)
    mat_new = np.c_[mat[:,2],mat[:,1],-mat[:,0],mat[:,3]]
    return transformations.quaternion_from_matrix(mat_new)

def normalize(v):
    return v / np.linalg.norm(v)

#def execute_dual_arm_trajectory(pr2, lquat, lpos, lgrip, left_used, rquat, rpos, rgrip, right_used):

    #if left_used: 
        #cmclient.switch_controllers(["l_cart"], ["l_arm_controller"])
    #else:
        #cmclient.switch_controllers(["l_arm_controller"], ["l_cart"])
        #brett.larm.goto_posture("side")

    #if right_used: 
        #cmclient.switch_controllers(["r_cart"], ["r_arm_controller"])
    #else:
        #cmclient.switch_controllers(["r_arm_controller"], ["r_cart"])
        #brett.rarm.goto_posture("side")


    #try: 
        #for i in xrange(len(lquat)):
            #brett.larm.set_cart_target(normalize(quatswap(lquat[i])), lpos[i], "/base_footprint")
            #brett.rarm.set_cart_target(normalize(quatswap(rquat[i])), rpos[i], "/base_footprint")
            #if lgrip[i] < .03: lgrip[i] = 0
            #if rgrip[i] < .03: rgrip[i] = 0
            #brett.lgrip.set_angle_target(lgrip[i])
            #brett.rgrip.set_angle_target(rgrip[i])

            #sleep(.04)
    #except Exception:
        #cmclient.switch_controllers(["l_arm_controller"], ["l_cart"])
        #cmclient.switch_controllers(["r_arm_controller"], ["r_cart"])
        #traceback.print_exc()
        #exit(0)

        

# TODO: set saturation arguments for velocity and torque
# also reduce max effort arguments

rospy.init_node("knot_tying",disable_signals=True)
comm.initComm();
rope_getter = comm.LatestFileGetter("rope_model","txt",comm.ArrayMessage)
library = rope_library.RopeTrajectoryLibrary(args.dbname, "write")
brett = pr2.PR2()
cmclient = ControllerManagerClient()

while True:
    rope_k3 = rope_getter.recv_latest().data
    draw_curve(rope_k3)

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
    execute_dual_arm_trajectory(pr2, l_quat, l_pos, l_grip/7, left_used, r_quat, r_pos, r_grip/7, right_used)
    print "done"
    
