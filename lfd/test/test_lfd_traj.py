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
parser.add_argument('traj_file')
parser.add_argument('--slice', default='')
parser.add_argument('--sim', action='store_true')
args = parser.parse_args()


if rospy.get_name() == "/unnamed":
    rospy.init_node("test_lfd_traj",disable_signals=True)
pr2 = PR2.create()
rospy.sleep(.5)

table_bounds = map(float, rospy.get_param("table_bounds").split())
kinbodies.create_box_from_bounds(pr2.env,table_bounds, name="table")

traj = None
with open(args.traj_file, 'r') as f:
    traj = pickle.load(f)

slice_start, slice_end = 0, -1
if args.slice != '':
    slice_start, slice_end = map(int, args.slice.split(':'))

if args.sim:
    import time
    import openravepy as rave
    env = rave.Environment() # create openrave environment
    env.SetViewer('qtcoin') # attach viewer (optional)
    env.Load('robots/pr2-beta-static.zae') # load a simple scene
    env.UpdatePublishedBodies()
    time.sleep(1)
    rave_pr2 = env.GetRobots()[0] # get the first robot

    with env: # lock the environment since robot will be used
        l_len, r_len = 0, 0
        leftarm = rave_pr2.GetManipulator('leftarm')
        rightarm = rave_pr2.GetManipulator('rightarm')
        if 'l_arm' in traj: l_len = len(traj['l_arm'])
        if 'r_arm' in traj: r_len = len(traj['r_arm'])
        if slice_end == -1: slice_end = max(l_len, r_len)
        for i in range(slice_start, min(slice_end, max(l_len, r_len))):
            if i < l_len: rave_pr2.SetDOFValues(traj['l_arm'][i,:], leftarm.GetArmIndices())
            if i < r_len: rave_pr2.SetDOFValues(traj['r_arm'][i,:], rightarm.GetArmIndices())
            env.UpdatePublishedBodies()
            time.sleep(0.1)
            print i

else:
    if slice_end != -1:
        traj = lfd_traj.slice_traj(traj, slice_start, slice_end)
    lfd_traj.follow_trajectory_with_grabs(pr2, traj, ignore_failure=True)
