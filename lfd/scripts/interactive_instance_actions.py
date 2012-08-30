#!/usr/bin/env python

"""
Interactively choose a verb, and then choose the targets, with the command line.
"""

from __future__ import division

import argparse
parser = argparse.ArgumentParser()
args = parser.parse_args()


import matplotlib
matplotlib.use('Qt4Agg')


import roslib
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import *
import rospy
from brett2.ros_utils import xyz2pc
import numpy as np
import h5py
import sensor_msgs.msg as sm
from brett2 import ros_utils, PR2
from point_clouds import tabletop
from jds_utils.yes_or_no import yes_or_no
import scipy.spatial.distance as ssd
from jds_utils.math_utils import norms
roslib.load_manifest("snazzy_msgs")
from snazzy_msgs.srv import *
import yaml
import os.path as osp
from lfd import lfd_traj, warping


action_db=h5py.File("/home/joschu/python/lfd/data/instance_actions.h5","r")
with open("/home/joschu/python/lfd/data/instance_action_info.yaml","r") as fh:
    action_info = yaml.load(fh)


def select_from_list(list):
    strlist = [str(item) for item in list]
    while True:
        print "choose from the following options:"
        print " ".join("(%s)"%item for item in strlist)
        resp = raw_input("?) ")
        if resp not in strlist:
            print "invalid response. try again."
        else:
            return list[strlist.index(resp)]


def make_and_execute_action(action, object_ids):
    if "expansion" in action_info[action]:
        raise NotImplementedError
    else:
        new_object_id = object_ids[0]
        new_object_pose = detection_info[xxx]


        best_cost = np.inf
        
        for demo in action_db[action]:

                
            for new_object_equivalent_pose in sample_equivalent_poses(new_object_pose, action_info[new_object_id]["symmetry_type"]):
                demo1 = demo
    
                new_obj_from_old_obj = calc_obj_transform(demo1["object_id"], new_object_id)
                
                scene_transform = compose_transforms(
                    affine_transform(pose_to_hmat(new_object_pose)),
                    new_obj_from_old_obj, 
                    affine_transform(np.linalg.inv(demo1["object_hmat"])))
                
                
                total_cost = 0
                total_cost += calculate_transform_cost(scene_transform)

                warped_demo = warping.transform_demo_with_fingertips(scene_transform, demo1)

                Globals.pr2.update_rave() 
                trajectory = {}
                
                for lr in "lr":
                    leftright = {"l":"left","r":"right"}[lr]
                    if demo1["arms_used"] in [lr, "b"]:
                        arm_traj, feas_inds = lfd_traj.make_joint_traj(warped_demo["%s_gripper_tool_frame"%lr]["position"], warped_demo["%s_gripper_tool_frame"%lr]["orientation"], demo1["%sarm"%leftright], Globals.pr2.robot.GetManipulator("%sarm"%leftright),"base_footprint","%s_gripper_tool_frame"%lr,2+16)
                        trajectory["%s_arm"%lr] = arm_traj
                        rospy.loginfo("left arm: %i of %i points feasible", len(feas_inds), len(arm_traj))
                        trajectory["%s_grab"%lr] = demo1["%s_gripper_joint"%lr] < .07
                        trajectory["%s_gripper"%lr] = warped_demo["%s_gripper_joint"%lr]
                        trajectory["%s_gripper"%lr][trajectory["%s_grab"%lr]] = 0
                        
                        total_cost += feasibility_cost(len(feas_inds) / len(arm_traj))
                
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_f = scene_transform
    






                    
class Globals:
    pr2 = None
    rviz = None
    handles = []
    isinstance(pr2, PR2.PR2)
    isinstance(rviz, ros_utils.RvizWrapper)
    if args.count_steps: stage = 0
    
    def __init__(self): raise

    @staticmethod
    def setup():
        if Globals.pr2 is None: Globals.pr2 = PR2.PR2.create()
        if Globals.rviz is None: Globals.rviz = ros_utils.RvizWrapper.create()
        load_table()


if rospy.get_name() == "/unnamed": 
    rospy.init_node("interactive_actions",disable_signals=True)
pr2 = PR2.PR2.create()
exec_svc = rospy.ServiceProxy("exec_verb_traj",ExecTrajectory)
detection_svc = rospy.ServiceProxy("xxx",XXX)


    
action_list = [action for action in action_info if "hidden" not in action_info[action]]
while True:

    pr2.rgrip.open()
    pr2.lgrip.open()
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')
    action = select_from_list(action_list)
    pr2.join_all()
    
    
    arg_names = action_info[action]["arg_names"]
    print "arguments: %s"%" ".join(arg_names)

    detection_info = detection_svc.call()
    
    
    obj_ids = []
    successful_detection = True
    for arg_name in arg_names:
        possible_ids = action_info[arg_name]["object_ids"]
        arg_obj_ids = set(possible_ids).intersection(set(detection_info["object_ids"]))
        if len(arg_obj_id) == 0:
            print "couldn't find %s"%arg_name
            successful_detection = False
        elif len(arg_obj_id) > 1:
            print "found multiple matches for %s in scene"%arg_obj_id
            successful_detection = False
        else:
            obj_ids.append(list(arg_obj_ids)[0])
    if successful_detection:
        make_and_execute_action(action, detection_info)   
        
                                            
    
