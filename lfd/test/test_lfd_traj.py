#!/usr/bin/env python

from brett2.PR2 import PR2
import numpy as np
import roslib; 
roslib.load_manifest('rospy'); import rospy
import pickle
import lfd_traj
from kinematics import kinbodies

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--traj_file', default='')
parser.add_argument('--warped_demo', default='')
parser.add_argument('--slice', default='')
parser.add_argument('--sim', action='store_true')
args = parser.parse_args()


if rospy.get_name() == "/unnamed":
    rospy.init_node("test_lfd_traj",disable_signals=True)
pr2 = PR2.create()

table_bounds = map(float, rospy.get_param("table_bounds").split())
kinbodies.create_box_from_bounds(pr2.env,table_bounds, name="table")

pr2.update_rave()

slice_start, slice_end = 0, -1
if args.slice != '':
    slice_start, slice_end = map(int, args.slice.split(':'))

traj, warped_demo = None, None
if args.traj_file:
    with open(args.traj_file, 'r') as f:
        traj = pickle.load(f)
else:
    with open(args.warped_demo, 'r') as f:
        warped_demo = pickle.load(f)
    traj = {}
    for lr in "lr":
        leftright = {"l":"left","r":"right"}[lr]
        #if best_demo["arms_used"] in [lr, "b"]:
        if True:
            arm_traj, feas_inds = lfd_traj.make_joint_traj_by_graph_search(
                warped_demo["%s_gripper_tool_frame"%lr]["position"],
                warped_demo["%s_gripper_tool_frame"%lr]["orientation"],
                pr2.robot.GetManipulator("%sarm"%leftright),
                "%s_gripper_tool_frame"%lr,
                check_collisions=True
            )
            if len(feas_inds) == 0: assert False and 'failure'
            traj["%s_arm"%lr] = arm_traj
            rospy.loginfo("%s arm: %i of %i points feasible", leftright, len(feas_inds), len(arm_traj))

if args.sim:
    import time
    pr2.env.SetViewer('qtcoin') # attach viewer (optional)
    #pr2.env.Load('robots/pr2-beta-static.zae') # load a simple scene
    pr2.env.UpdatePublishedBodies()
    time.sleep(1)
    rave_pr2 = pr2.robot

    if warped_demo:
        handles = []
        for lr in 'lr':
            if "%s_gripper_tool_frame"%lr in warped_demo:
                handles.append(pr2.env.plot3(points=warped_demo["%s_gripper_tool_frame"%lr]['position'], pointsize=1.0))

    with pr2.env: # lock the environment since robot will be used
        l_len, r_len = 0, 0
        leftarm = rave_pr2.GetManipulator('leftarm')
        rightarm = rave_pr2.GetManipulator('rightarm')
        if 'l_arm' in traj: l_len = len(traj['l_arm'])
        if 'r_arm' in traj: r_len = len(traj['r_arm'])
        if slice_end == -1: slice_end = max(l_len, r_len)
        for i in range(slice_start, min(slice_end, max(l_len, r_len))):
            if i < l_len: rave_pr2.SetDOFValues(traj['l_arm'][i,:], leftarm.GetArmIndices())
            if i < r_len: rave_pr2.SetDOFValues(traj['r_arm'][i,:], rightarm.GetArmIndices())
            pr2.env.UpdatePublishedBodies()
            time.sleep(0.1)
            print i

else:
    if slice_end != -1:
        traj = lfd_traj.slice_traj(traj, slice_start, slice_end)
    lfd_traj.follow_trajectory_with_grabs(pr2, traj, ignore_failure=True)
