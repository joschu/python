#!/usr/bin/env python

"""
Service for executing a one or two arm trajectory
"""


from __future__ import division
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
args = parser.parse_args()



import rospy
import itertools, os, lfd
from numpy import asarray
import numpy as np
import roslib
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import ExecTrajectoryRequest, ExecTrajectoryResponse, ExecTrajectory
from brett2.ros_utils import RvizWrapper, Marker
from brett2 import PR2
from brett2 import trajectories, ros_utils
from utils import conversions
import geometry_msgs.msg as gm
from kinematics import kinbodies
from point_clouds import tabletop
from utils.func_utils import once
import sensor_msgs.msg as sm
from lfd import lfd_traj as lt

class Globals:
    pr2 = None
    rviz = None
    isinstance(pr2, PR2.PR2)
    isinstance(rviz, ros_utils.RvizWrapper)
    
    def __init__(self): raise

    @staticmethod
    def setup():
        if Globals.pr2 is None: 
            Globals.pr2 = PR2.PR2.create()
        if Globals.rviz is None: Globals.rviz = ros_utils.RvizWrapper.create()
        load_table()
        #Globals.pr2.rarm.vel_limits[-1] *= 3
        #Globals.pr2.rarm.acc_limits[-1] *= 3
        #rospy.logwarn("warning: turning up roll link speed limits")

def load_table():
    table_bounds = map(float, rospy.get_param("table_bounds").split())
    kinbodies.create_box_from_bounds(Globals.pr2.env,table_bounds, name="table")
    

def callback(req):    
    Globals.handles = []
    
    traj = req.traj
    if traj.arms_used in "rl":
        lr = traj.arms_used
        manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
        manip = Globals.pr2.robot.GetManipulator(manip_name)
        gripper0_xyzs, gripper0_quats = [],[]
        for pose in traj.gripper0_poses.poses:
            xyz,quat = conversions.pose_to_trans_rot(pose)
            gripper0_xyzs.append(xyz)
            gripper0_quats.append(quat)
        joint_positions, inds = trajectories.make_joint_traj(gripper0_xyzs, gripper0_quats, manip, 'base_footprint', '%s_gripper_tool_frame'%lr, filter_options = 1)
        feasible_frac = len(inds)/len(gripper0_xyzs)
        best_feasible_frac, best_lr, best_joint_positions = feasible_frac, lr, joint_positions
        if best_feasible_frac < .7:
            rospy.logerr("unacceptably low fraction of ik success: %.4f",best_feasible_frac)
            return ExecTrajectoryResponse(success=False)

        rospy.loginfo("using %s hand", {"l":"left", "r":"right"}[best_lr])


        gripper_angles = np.array(traj.gripper0_angles)
        rospy.logwarn("warning! gripper angle hack")
        gripper_angles[gripper_angles < .04] = gripper_angles[gripper_angles < .04] - .02
    
        body_traj = {}
        body_traj["%s_arm"%best_lr] = best_joint_positions
        body_traj["%s_gripper"%best_lr] = gripper_angles
        pose_array = gm.PoseArray()
        pose_array.header.frame_id = "base_footprint"
        pose_array.poses = traj.gripper0_poses.poses
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1)))
        lt.follow_trajectory_with_grabs(Globals.pr2, body_traj)
        
    
    elif traj.arms_used in 'b':
        body_traj = {}
        for (lr,gripper_poses, gripper_angles) in zip("lr",[traj.gripper0_poses.poses,traj.gripper1_poses.poses], [traj.gripper0_angles,traj.gripper1_angles]):
            gripper_angles = np.array(gripper_angles)
            rospy.logwarn("warning! gripper angle hack")
            gripper_angles[gripper_angles < .04] = gripper_angles[gripper_angles < .04] - .02
            
            manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
            manip = Globals.pr2.robot.GetManipulator(manip_name)
            gripper_xyzs, gripper_quats = [],[]
            for pose in gripper_poses:
                xyz,quat = conversions.pose_to_trans_rot(pose)
                gripper_xyzs.append(xyz)
                gripper_quats.append(quat)
            joint_positions, inds = trajectories.make_joint_traj(gripper_xyzs, gripper_quats, manip, 'base_footprint', '%s_gripper_tool_frame'%lr, filter_options = 1)
            
            if len(inds) == 0:
                return ExecTrajectoryResponse(success=False)
             
            
            feasible_frac = len(inds)/len(gripper_xyzs)
            body_traj["%s_arm"%lr] = joint_positions
            body_traj["%s_gripper"%lr] = gripper_angles

        pose_array = gm.PoseArray()
        pose_array.header.frame_id = "base_footprint"
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = traj.gripper0_poses.poses
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1)))

        lt.follow_trajectory_with_grabs(pr2, dict(l_gripper = body_traj))
        lt.follow_trajectory_with_grabs(Globals.pr2, body_traj)

    else: raise NotImplementedError
    
    Globals.pr2.join_all()
    
    return ExecTrajectoryResponse(success=True)

    


if __name__ == "__main__":
    
    if args.test:        
        if rospy.get_name() == "/unnamed": 
            rospy.init_node("test_exec_verb_traj_service",disable_signals=True)
        Globals.setup()            
        import h5py
        F=h5py.File("/home/joschu/python/lfd/data/verbs.h5","r")
        demo = F["grab-cup"]["demos"]["0"]
        req = ExecTrajectoryRequest()
        traj = req.traj
        traj.gripper0_poses.poses = [conversions.trans_rot_to_pose(trans,rot) for (trans,rot) in zip(demo["l_gripper_xyzs"], demo["l_gripper_quats"])]
        traj.gripper0_angles = demo["l_gripper_angles"]
        traj.arms_used = demo["arms_used"].value
        callback(req)    
    else:
        rospy.init_node("exec_verb_traj_service",disable_signals=True)
        Globals.setup()
        service = rospy.Service("exec_verb_traj", ExecTrajectory, callback)
        rospy.spin()