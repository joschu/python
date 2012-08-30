#!/usr/bin/env python
"""
Take trajectory from bag file and put it in an hdf5 file
Eventually I should put the meat of this script into a function, because otherwise I have to do hacky shit with subprocess
"""
import argparse
from os.path import exists, join, dirname

parser = argparse.ArgumentParser()
parser.add_argument("bag")
parser.add_argument("h5file")
parser.add_argument("h5path")
parser.add_argument("arms_used",choices = ["r","l","b"],default="b")
args = parser.parse_args()
if not exists(args.bag):
    raise IOError("%s does not exist"%args.bag)

R_TOOL_FRAME = "r_gripper_tool_frame"
L_TOOL_FRAME = "l_gripper_tool_frame"
R1_FRAME = "r_gripper_r_finger_tip_link"
R2_FRAME = "r_gripper_l_finger_tip_link"
L1_FRAME = "l_gripper_r_finger_tip_link"
L2_FRAME = "l_gripper_l_finger_tip_frame"
REF_FRAME = "base_footprint"
MIN_TIME = .1
MIN_JOINT_CHANGE = .0025


import h5py
import numpy as np
from jds_utils.yes_or_no import yes_or_no
from jds_utils import conversions
import sys
import openravepy
import rosbag

if exists(args.h5file): filemode = "r+"
else: filemode = "w"
h5file = h5py.File(args.h5file,filemode)


if args.h5path in h5file:
    consent = yes_or_no("a group named %s already exists. delete it?"%args.h5path)
    if consent: del h5file[args.h5path]
    else: raise IOError
    
traj = h5file.create_group(args.h5path)
#traj["verb"] = "VERB"
#traj["verb_args"] = ["arg0", "arg1", "arg2"]


class RosToRave(object):
    def __init__(self):
        self.env = openravepy.Environment()
        self.env.Load("robots/pr2-beta-static.zae")
        self.robot = self.env.GetRobot("pr2")
        self.initialized = False
    def init(self, joint_msg):

        self.ros_names = joint_msg.name
        inds_ros2rave = np.array([self.robot.GetJointIndex(name) for name in self.ros_names])
        self.good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
        self.rave_inds = inds_ros2rave[self.good_ros_inds] # openrave indices corresponding to those joints

        ros_values = joint_msg.position        
        rave_values = [ros_values[i_ros] for i_ros in self.good_ros_inds]        
        self.robot.SetJointValues(rave_values[:20],self.rave_inds[:20])
        self.robot.SetJointValues(rave_values[20:],self.rave_inds[20:])          
        
        self.initialized = True

    def update(self, joint_msg):
        ros_values = joint_msg.position        
        rave_values = [ros_values[i_ros] for i_ros in self.good_ros_inds]        
        self.robot.SetJointValues(rave_values[:20],self.rave_inds[:20])
        self.robot.SetJointValues(rave_values[20:],self.rave_inds[20:])          


bag = rosbag.Bag(args.bag)
rr = RosToRave()
robot = rr.robot

traj["r_tool_frame"] = R_TOOL_FRAME
traj["l_tool_frame"] = L_TOOL_FRAME
traj["ref_frame"] = REF_FRAME
traj["bag_file"] = args.bag
traj["command"] = " ".join(sys.argv)
traj["arms_used"] = args.arms_used

r_tool_link = robot.GetLink(R_TOOL_FRAME)
r1_link = robot.GetLink(R1_FRAME)
r2_link = robot.GetLink(R2_FRAME)
l_tool_link = robot.GetLink(L_TOOL_FRAME)
l1_link = robot.GetLink(L1_FRAME)
l2_link = robot.GetLink(L2_FRAME)
r_gripper_joint = robot.GetJoint("r_gripper_joint")
l_gripper_joint = robot.GetJoint("l_gripper_joint")

joints = []
times = []
r_gripper_xyzs = []
r_gripper_xyzs1 = []
r_gripper_xyzs2 = []
r_gripper_quats = []
l_gripper_xyzs = []
l_gripper_xyzs1 = []
l_gripper_xyzs2 = []
l_gripper_quats = []
r_gripper_angles = []
l_gripper_angles = []

first_message = True
for (topic, msg, t) in bag.read_messages(topics=['/joint_states']):
    t = t.to_sec()
    if first_message:
        print "first message"
        rr.init(msg)
        traj["joint_names"] = msg.name
        prev_positions = np.ones(len(msg.position))*100
        prev_t = -np.inf
        first_message = False        
                
    if t - prev_t > MIN_TIME and np.any(np.abs(np.array(msg.position) - prev_positions) > MIN_JOINT_CHANGE):
        #print "adding data"
        
        rr.update(msg)
        
        
        joints.append(np.array(msg.position))
        times.append(t)
        
        if args.arms_used == "r" or args.arms_used == "b":
            hmat = r_tool_link.GetTransform()
            xyz, quat = conversions.hmat_to_trans_rot(hmat)        
            r_gripper_angles.append(r_gripper_joint.GetValues()[0])
            r_gripper_xyzs.append(xyz)
            r_gripper_xyzs1.append(r1_link.GetTransform()[:3,3])
            r_gripper_xyzs2.append(r2_link.GetTransform()[:3,3])
            r_gripper_quats.append(quat)            
        if args.arms_used == "l" or args.arms_used == "b":            
            hmat = l_tool_link.GetTransform()
            xyz, quat = conversions.hmat_to_trans_rot(hmat)    
            l_gripper_angles.append(l_gripper_joint.GetValues()[0])
            l_gripper_xyzs.append(xyz)
            l_gripper_xyzs1.append(l1_link.GetTransform()[:3,3])
            l_gripper_xyzs2.append(l2_link.GetTransform()[:3,3])
            l_gripper_quats.append(quat)  
            
        prev_t = t
        prev_positions = np.array(msg.position)
        
traj["joints"] = joints
traj["times"] = times
if args.arms_used == "r" or args.arms_used == "b":
    traj["r_gripper_xyzs"] = r_gripper_xyzs
    traj["r_gripper_xyzs1"] = r_gripper_xyzs1
    traj["r_gripper_xyzs2"] = r_gripper_xyzs2
    traj["r_gripper_quats"] = r_gripper_quats
    traj["r_gripper_angles"] = r_gripper_angles
if args.arms_used == "l" or args.arms_used == "b":
    traj["l_gripper_xyzs"] = l_gripper_xyzs
    traj["l_gripper_xyzs1"] = l_gripper_xyzs1
    traj["l_gripper_xyzs2"] = l_gripper_xyzs2
    traj["l_gripper_quats"] = l_gripper_quats
    traj["l_gripper_angles"] = l_gripper_angles


h5file.close()