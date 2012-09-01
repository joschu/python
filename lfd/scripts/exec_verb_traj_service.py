#!/usr/bin/env python

"""
Service for executing a one or two arm trajectory
"""
import rospy
from lfd import exec_verb_traj
from verb_msgs.srv import *


if __name__ == "__main__":
    rospy.init_node("exec_verb_traj_service",disable_signals=True)
    exec_verb_traj.Globals.setup()
    service = rospy.Service("exec_verb_traj", ExecTrajectory, exec_verb_traj.exec_traj)
    rospy.spin()