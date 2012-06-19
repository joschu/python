import argparse
parser = argparse.ArgumentParser()
parser.add_argument("file")
parser.add_argument("group")
args = parser.parse_args()

import h5py
from brett2.PR2 import PR2,IKFail
from brett2 import trajectories
from kinematics import kinematics_utils
import rospy
from brett2.ros_utils import RvizWrapper,Marker
from numpy import asarray
import numpy as np
import geometry_msgs.msg as gm
from utils import conversions

if rospy.get_name()=="/unnamed":
    rospy.init_node("playback_demo")

rviz = RvizWrapper.create()
pr2 = PR2.create()
rospy.sleep(1)


traj = h5py.File(args.file, 'r')[args.group]

ps = gm.PoseStamped(pose = gm.Pose(
    position = gm.Point(*traj["object_pose"]),
    orientation = gm.Quaternion(*traj["object_orientation"])))
ps.header.frame_id = 'base_link'
rviz.draw_marker(
    ps,
    id=1,
    type=Marker.CUBE,
    rgba = (0,1,0,1),
    scale = asarray(traj["object_dimension"]))

pose_array = conversions.array_to_pose_array(asarray(traj["gripper_positions"]), 'base_link')
rviz.draw_curve(
    pose_array,
    id=0)

n_waypoints = 20
xyzquat = np.c_[traj["gripper_positions"],traj["gripper_orientations"]]
xyzquat_rs = kinematics_utils.unif_resample(xyzquat, n_waypoints, weights = np.ones(7), tol=.001)
times = np.linspace(0,10,n_waypoints)

pr2.torso.go_up()
pr2.join_all()
pr2.update_rave()
joint_positions,_ = trajectories.make_joint_traj(xyzquat_rs[:,0:3], xyzquat_rs[:,3:7], pr2.rarm.manip, 'base_link', 'r_wrist_roll_link', filter_options = 18)
joint_velocities = kinematics_utils.get_velocities(joint_positions, times, tol=.001)
pr2.rarm.follow_timed_joint_trajectory(joint_positions, joint_velocities, times)

#for xyzq in xyzquat_rs:
    #xyz = xyzq[:3]
    #quat = xyzq[3:]
    #hmat = conversions.trans_rot_to_hmat(xyz,quat)
    #try:
        #pr2.rarm.goto_pose_matrix(hmat, 'base_link', 'r_wrist_roll_link')
        #pr2.join_all()
    #except IKFail:
        #pass

pr2.join_all()
                             
                                     