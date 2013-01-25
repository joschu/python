"""
This file contains the function exec_traj that will execute a trajectory on the robot specified by ExecTrajectoryRequest
"""

from __future__ import division
import rospy
import numpy as np
import roslib
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import ExecTrajectoryRequest, ExecTrajectoryResponse, ExecTrajectory
import brett2.ros_utils as ru
from brett2 import PR2
from jds_utils import conversions as juc
import geometry_msgs.msg as gm
from kinematics import kinbodies
import sensor_msgs.msg as sm
from lfd import lfd_traj as lt
from jds_utils.yes_or_no import yes_or_no
import scipy.spatial
import openravepy as rave
from jds_image_proc.clouds import voxel_downsample
from lfd import ik_functions, math_utils
roslib.load_manifest("bulletsim_msgs")
from bulletsim_msgs.srv import PlanTraj, PlanTrajRequest, PlanTrajResponse

class Globals:
    handles = []
    pr2 = None
    rviz = None
    isinstance(pr2, PR2.PR2)
    isinstance(rviz, ru.RvizWrapper)
    table_loaded = False
    
    def __init__(self): raise

    @staticmethod
    def setup():
        Globals.pr2 = PR2.PR2.create()
        Globals.rviz = ru.RvizWrapper.create()
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
    ps = gm.PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.pose.position = gm.Point(*aabb.pos())
    ps.pose.orientation = gm.Quaternion(0,0,0,1)
    Globals.handles.append(Globals.rviz.draw_marker(ps, type=ru.Marker.CUBE, scale = aabb.extents()*2, id = 24019,rgba = (1,0,0,.25)))
    
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

# checks if the gripper is closing during the stage
def is_closing(angles):
    return get_close_index(angles) < len(angles)

# checks if the gripper is opening during the stage
def is_opening(angles):
    return get_open_index(angles) < len(angles)

# replace the slice of old_array between start and end with new_value
def replace_range(new_value, old_array, start=0, end=-1):
    end = len(old_array) if end == -1 else end
    return np.concatenate((old_array[:start], np.array([new_value for i in xrange(end-start)]), old_array[end:]))

# if the gripper effort is this much or more (by magnitude), then it is grabbing something
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

# fixes the end of joint_positions once the gripper starts opening or closing
# adds the point cloud to the openrave environment
def setup_obj_rave(obj_cloud_xyz, obj_name):
    rave_env = Globals.pr2.env
    pc_down = voxel_downsample(obj_cloud_xyz, .02)
    body = rave.RaveCreateKinBody(rave_env, '')
    body.SetName(obj_name) #might want to change this name later
    delaunay = scipy.spatial.Delaunay(pc_down)
    body.InitFromTrimesh(rave.TriMesh(delaunay.points, delaunay.convex_hull), True)
    rave_env.Add(body)
    return body
 
# have the robot grab the openrave object
def grab_obj_single_grip_rave(lr, obj_kinbody):
    rave_robot = Globals.pr2.robot
    rave_robot.Grab(obj_kinbody, rave_robot.GetLink("%s_gripper_tool_frame"%lr))
    
# have the robot release what it has grabbed
# assuming that it has only grabbed the tool, so that is the only thing being released
def release_objs_single_grip_rave(lr):
    rave_robot = Globals.pr2.robot
    rave_robot.ReleaseAllGrabbed()

# grab or release the grab_obj_kinbody depending on if the gripper is opening or closing in the stage
def handle_grab_or_release_obj(grab_obj_kinbody, l_gripper_poses, l_gripper_angles, r_gripper_poses, r_gripper_angles):
    if len(l_gripper_angles) > 0:
        if is_closing(l_gripper_angles):
            grab_obj_single_grip_rave('l', grab_obj_kinbody)
        elif is_opening(l_gripper_angles):
            release_objs_single_grip_rave('l')
    if len(r_gripper_angles) > 0:
        if is_closing(r_gripper_angles):
            grab_obj_single_grip_rave('r', grab_obj_kinbody)
        elif is_opening(r_gripper_angles):
            release_objs_single_grip_rave('r')

# linearly interpolates between the start and the end positions
# start and end pos are hmats
def get_lin_interp_poses(start_pos, end_pos, n_steps):
    xyz_start, quat_start = juc.hmat_to_trans_rot(start_pos)
    xyz_end, quat_end = juc.hmat_to_trans_rot(end_pos)
    xyzs = math_utils.linspace2d(xyz_start, xyz_end, n_steps)
    quats = [rave.quatSlerp(quat_start, quat_end, t) for t in np.linspace(0,1,n_steps)]
    hmats = [rave.matrixFromPose(np.r_[quat[3],quat[:3],xyz]) for (xyz, quat) in zip(xyzs, quats)]
    gripper_poses = np.array([juc.hmat_to_pose(hmat) for hmat in hmats])
    return gripper_poses

# prepends linearly interpolated path from current_pos to the start of the current path
def prepend_path_to_start(current_pos, gripper_poses, gripper_angles, n_steps):
    before_traj = get_lin_interp_poses(current_pos, juc.pose_to_hmat(gripper_poses[0]), n_steps)
    prepended_gripper_poses = np.concatenate((np.array(before_traj), gripper_poses))
    updated_angles = np.concatenate((np.ones(n_steps) * gripper_angles[0], gripper_angles))
    return prepended_gripper_poses, updated_angles
    
# remove poses lower than the current position at the beginning of the trajectory 
def remove_lower_poses(current_pos, gripper_poses, gripper_angles):
    xyz, quat = juc.hmat_to_trans_rot(current_pos)
    z_floor = xyz[2]
    low_index = 0
    for pose in gripper_poses:
        if pose.position.z >= z_floor:
            break
        low_index += 1
    removed_lower_poses = gripper_poses[low_index:]
    updated_angles = gripper_angles[low_index:]
    return removed_lower_poses, updated_angles

# wrapper for exec_traj_do_work which takes in a ros message
def exec_traj(req, traj_ik_func=ik_functions.do_traj_ik_graph_search, obj_pc=None, obj_name=""):
    assert isinstance(req, ExecTrajectoryRequest)
    traj = req.traj
    return exec_traj_do_work(traj.l_gripper_poses.poses, traj.l_gripper_angles,
                             traj.r_gripper_poses.poses, traj.r_gripper_angles,
                             traj_ik_func, ros_utils.pc2xyzrgb(obj_pc)[0], obj_name)

# unwrap continuous joints to avoid winding
def unwrap_angles(angles):
    for joint_num in [2, 4, 6]:
        angles[:, joint_num] = np.unwrap(angles[:, joint_num])
    return angles

def exec_traj_do_work(l_gripper_poses, l_gripper_angles, r_gripper_poses, r_gripper_angles, traj_ik_func, obj_cloud_xyz, obj_name, can_move_lower=True):
    del Globals.handles[1:]
    grab_obj_kinbody = setup_obj_rave(obj_cloud_xyz, obj_name) if obj_cloud_xyz is not None else None
    body_traj = {}
    for (lr, gripper_poses, gripper_angles) in zip("lr", [l_gripper_poses, r_gripper_poses], [l_gripper_angles, r_gripper_angles]):
        if len(gripper_poses) == 0: continue

        gripper_poses, gripper_angles = np.array(gripper_poses), np.array(gripper_angles)

        gripper_angles_grabbing = process_gripper_angles_for_grabbing(lr, gripper_angles)

        Globals.pr2.update_rave()
        current_pos = Globals.pr2.robot.GetLink("%s_gripper_tool_frame"%lr).GetTransform()

        if can_move_lower:
            gripper_poses_remove, gripper_angles_remove = gripper_poses, gripper_angles_grabbing
        else:
            gripper_poses_remove, gripper_angles_remove = remove_lower_poses(current_pos, gripper_poses, gripper_angles_grabbing)

        gripper_poses_prepend, gripper_angles_prepend = prepend_path_to_start(current_pos, gripper_poses_remove, gripper_angles_remove, 15)

        final_gripper_poses, final_gripper_angles = gripper_poses_prepend, gripper_angles_prepend

        #do ik
        joint_positions = traj_ik_func(Globals.pr2, lr, final_gripper_poses)
        if len(joint_positions) == 0:
            return ExecTrajectoryResponse(success=False)

        unwrapped_joint_positions = unwrap_angles(joint_positions)
        final_joint_positions = unwrapped_joint_positions

        body_traj["%s_arm"%lr] = final_joint_positions
        body_traj["%s_gripper"%lr] = final_gripper_angles

    yn = yes_or_no("continue?")
    if yn:
        lt.follow_trajectory_with_grabs(Globals.pr2, body_traj)

        if grab_obj_kinbody is not None:
            handle_grab_or_release_obj(grab_obj_kinbody, l_gripper_poses, l_gripper_angles, r_gripper_poses, r_gripper_angles)

        raw_input("Press enter when done viewing trajectory")

        Globals.pr2.join_all()

        return ExecTrajectoryResponse(success=True)
    else:
        return ExecTrajectoryResponse(success=False)
