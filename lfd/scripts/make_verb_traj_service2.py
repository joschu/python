#!/usr/bin/env python

"""
Given a verb and a bunch of point clouds, spits out a trajectory
"""
import rospy
from verb_msgs.srv import *
from lfd import make_verb_traj

if __name__ == "__main__":
    rospy.init_node("make_verb_traj_service",disable_signals=True)
    make_verb_traj.Globals.setup()
    service = rospy.Service("make_verb_traj", MakeTrajectory,make_verb_traj.make_traj)
    rospy.spin()