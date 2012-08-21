#!/usr/bin/env python

import lfd
from lfd import bag_proc
import rosbag
import os.path as osp
import os
import yaml
import h5py
from copy import copy

data_dir = osp.join(osp.dirname(lfd.__file__), "data")
TASK_FILE = osp.join(data_dir, "instance_action_demos.yaml")
VERB_LIB_PATH = osp.join(data_dir, "instance_actions.h5")

if osp.exists(LIB_PATH): os.remove(LIB_PATH)

with open(osp.join(data_dir,TASK_FILE),"r") as fh:
    demo_infos = yaml.load(fh)

task_lib = h5py.File(LIB_PATH, mode="w")

link_names = ["r_gripper_tool_frame", "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 
              "l_gripper_tool_frame", "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link"]

for (i_demo,demo_info) in enumerate(demo_infos):
    bag = rosbag.Bag(demo_info["bag_file"])

    
    kinematics_info = bag_proc.extract_kinematics_from_bag(bag, link_names)    
    detection_info = json.load(demo_info["detection_file"])

    
    action_info = copy(kinematics_info)
    action_info["object_id"] = detection_info[xxx]
    action_info["object_pose"] = detection_info[xxx]
    action_info["arms_used"] = bag_proc.determine_arms_used(kinematics_info)
    
    if action not in task_lib:
        task_lib.create_group(action)
    path = "%s/%.2i"%(action,len(task_lib[action]))
    bag_proc.dict_to_hdf(task_lib, action_info, path)