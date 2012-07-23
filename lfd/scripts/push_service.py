#!/usr/bin/env python
"""
Deprecated
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("file")
parser.add_argument("group")
args = parser.parse_args()

import h5py
from brett2.PR2 import PR2,IKFail
from brett2 import trajectories
from kinematics import kinematics_utils, retiming
import rospy
from brett2.ros_utils import RvizWrapper,Marker, pc2xyzrgb
from numpy import asarray
import numpy as np
import geometry_msgs.msg as gm
from utils import conversions
from lfd import registration
import roslib; roslib.load_manifest("verb_msgs")

from verb_msgs.srv import *


class Globals:
    pr2 = None
    rviz = None
    
    def __init__(self): raise

    @staticmethod
    def setup():
        Globals.pr2 = PR2.create()
        Globals.rviz = RvizWrapper.create()
        rospy.sleep(1)


OLD_ID = 0
NEW_ID = 1
COLORS = [(0,0,1,1), (0,1,0,1)]
def show_object_and_trajectory(traj_xyz, obj_xyz, obj_quat, obj_dim, id):
    ps = gm.PoseStamped(pose = gm.Pose(
        position = gm.Point(*obj_xyz),
        orientation = gm.Quaternion(*obj_quat)))
    ps.header.frame_id = 'base_footprint'
    HANDLES.append(Globals.rviz.draw_marker(
        ps,
        type=Marker.CYLINDER,
        rgba = COLORS[id],
        scale = asarray(obj_dim)))
    pose_array = conversions.array_to_pose_array(asarray(traj_xyz), 'base_footprint')
    HANDLES.append(Globals.rviz.draw_curve(
        pose_array,
        rgba = COLORS[id]))



if __name__ == "__main__":
    HANDLES = []
    rospy.init_node("push_service")
    Globals.setup()

    traj = h5py.File(args.file, 'r')[args.group]
    n_waypoints = 20
    xyzquat = np.c_[traj["gripper_positions"],traj["gripper_orientations"]]
    xyzquat = kinematics_utils.unif_resample(xyzquat, n_waypoints, weights = np.r_[1,1,1,.25,.25,.25,.25], tol=.001)
    old_xyzs, old_quats = xyzquat[:,:3], xyzquat[:,3:]
    times = np.linspace(0,10,n_waypoints)

    Globals.pr2.update_rave()
    
    f = registration.ThinPlateSpline()
    obj_xyz = asarray(traj["object_pose"])
    obj_quat = asarray(traj["object_orientation"])
    obj_dim = asarray(traj["object_dimension"])

    

    def callback(request):  
        global HANDLES
        HANDLES = []
        xyzs, _ = pc2xyzrgb(request.point_cloud)
        new_mins = xyzs.reshape(-1,3).min(axis=0)
        new_maxes = xyzs.reshape(-1,3).max(axis=0)
        new_obj_xyz = (new_mins + new_maxes) / 2
        new_obj_dim = new_maxes - new_mins
        f.fit(np.atleast_2d(obj_xyz), np.atleast_2d(new_obj_xyz), .001, .001)
    
        new_xyzs, new_mats = f.transform_frames(old_xyzs, conversions.quats2mats(old_quats))
        new_quats = conversions.mats2quats(new_mats)
    
        show_object_and_trajectory(new_xyzs, new_obj_xyz, obj_quat, new_obj_dim, NEW_ID)
        show_object_and_trajectory(xyzquat[:,:3], obj_xyz, obj_quat, obj_dim, OLD_ID)
        
        Globals.pr2.update_rave()        
        joint_positions,inds = trajectories.make_joint_traj(new_xyzs, new_quats, Globals.pr2.rarm.manip, 'base_link', 'r_wrist_roll_link', filter_options = 18)
        response = PushResponse()
        
        if joint_positions is not None:
            Globals.pr2.rarm.goto_joint_positions(joint_positions[0])
            #current_joints = Globals.pr2.rarm.get_joint_positions()[None,:]
            joint_positions, joint_velocities, times = retiming.make_traj_with_limits(
                joint_positions, Globals.pr2.rarm.vel_limits, Globals.pr2.rarm.acc_limits,smooth=True)
            #joint_velocities = kinematics_utils.get_velocities(joint_positions, times, tol=.001)

            Globals.pr2.rarm.follow_timed_joint_trajectory(joint_positions, joint_velocities, times)        
            
            response.success = True
        else:
            response.success = False
        return response
        
    rospy.Service("push", Push, callback)
    rospy.loginfo("ready to push some shit")
    rospy.spin()
