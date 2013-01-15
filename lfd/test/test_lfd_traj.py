#!/usr/bin/env python

from brett2.PR2 import PR2
import numpy as np
import roslib; 
roslib.load_manifest('rospy'); import rospy
import pickle
import lfd_traj
import traj_ik_graph_search
from kinematics import kinbodies
import jds_utils.conversions as conv
import openravepy as rave

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('--traj_file', default='')
parser.add_argument('--warped_demo', default='')
parser.add_argument('--slice', default='')
parser.add_argument('--sim', action='store_true')
parser.add_argument('--no_table', action='store_true')
args = parser.parse_args()


np.set_printoptions(linewidth=1000, threshold='nan')

if rospy.get_name() == "/unnamed":
    rospy.init_node("test_lfd_traj", disable_signals=True)
pr2 = PR2.create()

if not args.no_table:
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
        print traj
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

def show_ik_solns(part_name, manip, i, warped_demo):
    frame = "%s_gripper_tool_frame"%part_name[0]
    hmat = conv.trans_rot_to_hmat(warped_demo[frame]['position'][i], warped_demo[frame]['orientation'][i])
    solns = traj_ik_graph_search.ik_for_link(hmat, manip, frame, return_all_solns=True)

    # newrobots = []
    print 'Displaying', len(solns), 'IK solutions for', part_name, 'at time', i
    print solns
    # for i, s in enumerate(solns):
    #     # newrobot = rave.RaveCreateRobot(pr2.env, pr2.robot.GetXMLId())
    #     # newrobot.Clone(pr2.robot,0)
    #     # for link in newrobot.GetLinks():
    #     #     for geom in link.GetGeometries():
    #     #         geom.SetTransparency(0.5)
    #     # newrobots.append(newrobot)
    #     # pr2.env.Add(newrobot, True)
    #     # newrobot.SetTransform(pr2.robot.GetTransform())
    #     # newrobot.SetDOFValues(s, manip.GetArmIndices())
    #     pr2.robot.SetDOFValues(s, manip.GetArmIndices())
    #     pr2.env.UpdatePublishedBodies()
    #     print i, '/', len(solns),
    #     raw_input(' >')
    # # for newrobot in newrobots:
    #     pr2.env.Remove(newrobot)


def check_discont(traj, part_name, manip, i, warped_demo):
    maxgap = abs(traj[part_name][i,:] - traj[part_name][i-1,:]).max()
    if maxgap <= 0.1:
        return
    print 'WARNING: discont in', part_name, 'from', i-1, 'to', i, '| max gap =', maxgap
    print '\tvals at t =', i-1, traj[part_name][i-1,:]
    print '\tvals at t =', i, traj[part_name][i,:]
    print '\tjoint 6:', traj[part_name][i-1:i+1,6]
    print '\tunwrapped joint 6:', np.unwrap(traj[part_name][i-1:i+1,6])
    if warped_demo is not None:
        show_ik_solns(part_name, manip, i-1, warped_demo)
        show_ik_solns(part_name, manip, i, warped_demo)

if args.sim:
    import time
    pr2.env.SetViewer('qtcoin')
    pr2.env.UpdatePublishedBodies()
    time.sleep(1)
    rave_pr2 = pr2.robot

    if warped_demo:
        handles = []
        for lr in 'lr':
            if "%s_gripper_tool_frame"%lr in warped_demo:
                handles.append(pr2.env.plot3(points=warped_demo["%s_gripper_tool_frame"%lr]['position'], pointsize=1.0))

    with pr2.env:
        l_len, r_len = 0, 0
        leftarm = rave_pr2.GetManipulator('leftarm')
        rightarm = rave_pr2.GetManipulator('rightarm')
        if 'l_arm' in traj: l_len = len(traj['l_arm'])
        if 'r_arm' in traj: r_len = len(traj['r_arm'])
        if slice_end == -1: slice_end = max(l_len, r_len)

        start = slice_start
        end = min(slice_end, max(l_len, r_len))
        i = start
        while i < end:
            if i < l_len:
                if i > 0: check_discont(traj, 'l_arm', leftarm, i, warped_demo)
                rave_pr2.SetDOFValues(traj['l_arm'][i,:], leftarm.GetArmIndices())
            if i < r_len:
                if i > 0: check_discont(traj, 'r_arm', rightarm, i, warped_demo)
                rave_pr2.SetDOFValues(traj['r_arm'][i,:], rightarm.GetArmIndices())
            pr2.env.UpdatePublishedBodies()
            #time.sleep(0.1)
            print i
            inp = raw_input('.')
            if inp == 'p': i -= 1
            elif len(inp) >= 2 and inp[0] == 'j': i = int(inp[1:])
            else: i += 1

else:
    if slice_end != -1:
        traj = lfd_traj.slice_traj(traj, slice_start, slice_end)
    lfd_traj.follow_trajectory_with_grabs(pr2, traj, ignore_failure=True)
