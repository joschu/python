"""
This file contains the function exec_traj that will execute a trajectory on the robot specified by ExecTrajectoryRequest
"""

from __future__ import division
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
from jds_utils import conversions as juc
import geometry_msgs.msg as gm
from kinematics import kinbodies
from point_clouds import tabletop
from jds_utils.func_utils import once
import sensor_msgs.msg as sm
from lfd import lfd_traj as lt
from jds_utils.yes_or_no import yes_or_no
import kinematics.kinematics_utils as ku

import scipy.spatial
import openravepy as rave
from jds_image_proc.clouds import voxel_downsample

from lfd import ik_functions, math_utils

from lfd import make_verb_traj
roslib.load_manifest("bulletsim_msgs")
from bulletsim_msgs.srv import PlanTraj, PlanTrajRequest, PlanTrajResponse

class Globals:
    handles = []
    pr2 = None
    rviz = None
    isinstance(pr2, PR2.PR2)
    isinstance(rviz, ros_utils.RvizWrapper)
    table_loaded = False
    
    def __init__(self): raise

    @staticmethod
    def setup():
        Globals.pr2 = PR2.PR2.create()
        Globals.rviz = ros_utils.RvizWrapper.create()
        if not Globals.table_loaded:
            load_table()
            draw_table()

def load_table():
    while not rospy.has_param("table_bounds"):
        rospy.logwarn("waiting until table_bounds set")
        rospy.sleep(1)
    table_bounds = map(float, rospy.get_param("table_bounds").split())
    kinbodies.create_box_from_bounds(Globals.pr2.env,table_bounds, name="table")
    
    
def draw_table():
    aabb = Globals.pr2.robot.GetEnv().GetKinBody("table").GetLinks()[0].ComputeAABB()
    ps =gm.PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.pose.position = gm.Point(*aabb.pos())
    ps.pose.orientation = gm.Quaternion(0,0,0,1)
    Globals.handles.append(Globals.rviz.draw_marker(ps, type=Marker.CUBE, scale = aabb.extents()*2, id = 24019,rgba = (1,0,0,.25)))
    
# if the difference between two gripper angles is greater than this, then the gripper angle is "changing"
GRIPPER_ANGLE_TOLERANCE = 0.0005

# find where the gripper started to close
def get_close_index(angles):
    if len(angles) == 0:
        return 0
    start = angles[0]
    index = 0
    while index < len(angles):
        if (start - angles[index]) > GRIPPER_ANGLE_TOLERANCE:
            break;
        index += 1
    return index

# find where the gripper started to open
def get_open_index(angles):
    if len(angles) == 0:
        return 0
    start = angles[0]
    index = 0
    while index < len(angles):
        if (angles[index] - start) > GRIPPER_ANGLE_TOLERANCE:
            break;
        index += 1
    return index

def is_closing(angles):
    return get_close_index(angles) < len(angles)

def is_opening(angles):
    return get_open_index(angles) < len(angles)

def replace_range(new_value, old_array, start=0, end=-1):
    end = len(old_array) if end == -1 else end
    return np.concatenate((old_array[:start], np.array([new_value for i in xrange(end-start)]), old_array[end:]))

GRIPPER_EFFORT_GRAB_THRESHHOLD = -10
# gripper is 'l' or 'r'
def is_grabbing_obj(gripper):
    gripper_effort = Globals.pr2.rgrip.get_effort() if gripper == 'r' else Globals.pr2.lgrip.get_effort()
    return gripper_effort < GRIPPER_EFFORT_GRAB_THRESHHOLD #effort is negative
    
# if a gripper is to close, make sure it closes; if a gripper is already closed, make sure it stays closed
def process_gripper_angles_for_grabbing(lr, gripper_angles):
    if is_grabbing_obj(lr):
        start_open_index = get_open_index(gripper_angles)
        if start_open_index < len(gripper_angles):
            return replace_range(0, gripper_angles, end=start_open_index)
    else:
        start_close_index = get_close_index(gripper_angles)
        if start_close_index < len(gripper_angles):
            return replace_range(0, gripper_angles, start=start_close_index)
    return gripper_angles

def fix_end_joint_positions(lr, gripper_angles, joint_positions):
    if is_grabbing_obj(lr):
        start_open_index = get_open_index(gripper_angles)
        if 0 < start_open_index < len(joint_positions):
            return replace_range(joint_positions[start_open_index-1], joint_positions, start=start_open_index)
    else:
        start_close_index = get_close_index(gripper_angles)
        if 0 < start_close_index < len(joint_positions):
            return replace_range(joint_positions[start_close_index-1], joint_positions, start=start_close_index)
    return joint_positions

def plot_gripper_xyzs_from_poses(lr, gripper_poses):
    gripper_xyzs, gripper_quats = [],[]
    for pose in gripper_poses:
        xyz, quat = juc.pose_to_trans_rot(pose)
        gripper_xyzs.append(xyz)
        gripper_quats.append(quat)
    make_verb_traj.plot_curve(gripper_xyzs, (1, 1, 0, 1))

# adds the point cloud to the openrave environment
def setup_obj_rave(obj_pc, obj_name):
    rave_env = Globals.pr2.env
    pc_down = voxel_downsample(ros_utils.pc2xyzrgb(obj_pc)[0], .02)
    body = rave.RaveCreateKinBody(rave_env, '')
    body.SetName(obj_name) #might want to change this name later
    delaunay = scipy.spatial.Delaunay(pc_down)
    body.InitFromTrimesh(rave.TriMesh(delaunay.points, delaunay.convex_hull), True)
    rave_env.Add(body)
    return body
 
def grab_obj_single_grip_rave(lr, obj_kinbody):
    rave_robot = Globals.pr2.robot
    rave_robot.Grab(obj_kinbody, rave_robot.GetLink("%s_gripper_tool_frame"%lr))
    
# assuming now that only one object is held at a time
def release_objs_single_grip_rave(lr):
    rave_robot = Globals.pr2.robot
    rave_robot.ReleaseAllGrabbed()

# grab or release the grab_obj_kinbody depending on if the gripper is opening or closing in the stage
def handle_grab_or_release_obj(grab_obj_kinbody, req_traj):
    if len(req_traj.l_gripper_angles) > 0:
        if is_closing(req_traj.l_gripper_angles):
            grab_obj_single_grip_rave('l', grab_obj_kinbody)
        elif is_opening(req_traj.l_gripper_angles):
            release_objs_single_grip_rave('l')
    if len(req_traj.r_gripper_angles) > 0:
        if is_closing(req_traj.r_gripper_angles):
            grab_obj_single_grip_rave('r', grab_obj_kinbody)
        elif is_opening(req_traj.r_gripper_angles):
            release_objs_single_grip_rave('r')

# linearly interpolates between the start and the end positions
# start and end pos are hmats
def get_lin_interp_poses(start_pos, end_pos, n_steps):
    xyz_start, quat_start = juc.hmat_to_trans_rot(start_pos)
    xyz_end, quat_end = juc.hmat_to_trans_rot(end_pos)
    xyzs = math_utils.linspace2d(xyz_start, xyz_end, n_steps)
    quats = [rave.quatSlerp(quat_start, quat_end, t) for t in np.linspace(0,1,n_steps)]
    hmats = [rave.matrixFromPose(np.r_[quat[3],quat[:3],xyz]) for (xyz, quat) in zip(xyzs, quats)]
    gripper_poses = [juc.hmat_to_pose(hmat) for hmat in hmats]   
    return gripper_poses

def exec_traj(req, traj_ik_func=ik_functions.do_traj_ik_graph_search, obj_pc=None, obj_name=""):
    assert isinstance(req, ExecTrajectoryRequest)
    del Globals.handles[1:]
    
    grab_obj_kinbody = setup_obj_rave(obj_pc, obj_name) if obj_pc is not None else None

    traj = req.traj

    n_steps = 15

    body_traj = {}
    for (lr, gripper_poses, gripper_angles) in zip("lr", [traj.l_gripper_poses.poses, traj.r_gripper_poses.poses], [traj.l_gripper_angles, traj.r_gripper_angles]):
        if len(gripper_poses) == 0: continue

        unprocessed_gripper_angles = np.array(gripper_angles)
        #rospy.logwarn("warning! gripper angle hacks")
        #gripper_angles[gripper_angles < .04] = gripper_angles[gripper_angles < .04] - .02

        gripper_angles_grabbing = process_gripper_angles_for_grabbing(lr, unprocessed_gripper_angles)

        plot_gripper_xyzs_from_poses(lr, gripper_poses)

        # add poses to traj to get from current position to start of next traj
        Globals.pr2.update_rave()
        current_pos = Globals.pr2.robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()
        before_traj = get_lin_interp_poses(current_pos, juc.pose_to_hmat(gripper_poses[0]), n_steps)
        final_gripper_poses = np.concatenate((np.array(before_traj), gripper_poses))
        final_gripper_angles = np.concatenate((np.ones(n_steps)*gripper_angles_grabbing[0], gripper_angles_grabbing))

        #do ik
        joint_positions = traj_ik_func(Globals.pr2, lr, final_gripper_poses)

        if len(joint_positions) == 0:
            return ExecTrajectoryResponse(success=False)
        joint_positions = ku.smooth_positions(joint_positions, .15)
        final_joint_positions = fix_end_joint_positions(lr, unprocessed_gripper_angles, joint_positions)
        
        body_traj["%s_arm"%lr] = final_joint_positions
        body_traj["%s_gripper"%lr] = final_gripper_angles

        pose_array = gm.PoseArray()
        pose_array.header.frame_id = "base_footprint"
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = traj.r_gripper_poses.poses if lr == 'r' else traj.r_gripper_poses.poses
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1)))

    yn = yes_or_no("continue?")
    if yn:
        lt.follow_trajectory_with_grabs(Globals.pr2, body_traj)
        
        if grab_obj_kinbody is not None:
            handle_grab_or_release_obj(grab_obj_kinbody, traj)

        Globals.pr2.join_all()
        return ExecTrajectoryResponse(success=True)
    else:
        return ExecTrajectoryResponse(success=False)
    
