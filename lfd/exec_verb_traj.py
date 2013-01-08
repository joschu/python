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
from jds_utils import conversions
import geometry_msgs.msg as gm
from kinematics import kinbodies
from point_clouds import tabletop
from jds_utils.func_utils import once
import sensor_msgs.msg as sm
from lfd import lfd_traj as lt
from jds_utils.yes_or_no import yes_or_no
import kinematics.kinematics_utils as ku

from lfd import traj_ik_graph_search

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

def get_manipulator(lr):
    manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
    manip = Globals.pr2.robot.GetManipulator(manip_name)
    return manip

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
    last_start_index = 0
    while last_start_index < len(angles):
        if (start - angles[last_start_index]) > GRIPPER_ANGLE_TOLERANCE:
            break;
        last_start_index += 1
    return last_start_index

# find where the gripper started to open
def get_open_index(angles):
    if len(angles) == 0:
        return 0
    start = angles[0]
    last_start_index = 0
    while last_start_index < len(angles):
        if (angles[last_start_index] - start) > GRIPPER_ANGLE_TOLERANCE:
            break;
        last_start_index += 1
    return last_start_index

# change the starting angle of the gripper to new_angle
def replace_start_gripper_angle(new_angle, old_angles):
    last_start_index = get_close_index(old_angles)
    return np.concatenate((np.array([new_angle for i in xrange(last_start_index)]), old_angles[last_start_index:]))
    
# once the gripper starts to close, then force it to close all the way
# fixes the problem of objects being dropped because they are too small
# this is a hack; this can be done a better way
def full_gripper_close(angles):
    if len(angles) == 0:
        return angles
    last_start_index = get_close_index(angles)
    return np.concatenate((angles[:last_start_index], np.zeros(len(angles)-last_start_index)))

GRIPPER_EFFORT_GRAB_THRESHHOLD = -10

# if a gripper should be grabbing an object, make sure it is grabbing the object
def get_proper_gripper_angles(lr, gripper_angles):
    #gripper_effort = Globals.pr2.rgrip.get_effort() if lr == 'r' else Globals.pr2.lgrip.get_effort()
    #if gripper_effort < GRIPPER_EFFORT_GRAB_THRESHHOLD:

    if lr == 'r':
        current_gripper_angle = Globals.pr2.rgrip.get_angle()
    elif lr == 'l':
        current_gripper_angle = Globals.pr2.lgrip.get_angle()
    new_gripper_angles = replace_start_gripper_angle(current_gripper_angle, gripper_angles)
    new_gripper_angles = full_gripper_close(new_gripper_angles)
    return new_gripper_angles

# do ik using the graph search algorithm
def do_traj_ik_graph_search(lr, gripper_poses):
    hmats = [conversions.pose_to_hmat(pose) for pose in gripper_poses]

    def ikfunc(hmat):
        return traj_ik_graph_search.ik_for_link(hmat, manip, "%s_gripper_tool_frame"%lr, return_all_solns=True)

    manip = get_manipulator(lr)
    start_joints = Globals.pr2.robot.GetDOFValues(manip.GetArmJoints())

    def nodecost(joints):
        robot = manip.GetRobot()
        robot.SetDOFValues(joints, manip.GetArmJoints())
        return 100*robot.GetEnv().CheckCollision(robot)                

    paths, costs, timesteps = traj_ik_graph_search.traj_cart2joint(hmats, ikfunc, start_joints=start_joints, nodecost=nodecost)

    i_best = np.argmin(costs)
    print "lowest cost of initial trajs:", costs[i_best]
    path_init = paths[i_best]
    return path_init

# do ik using the openrave ik
def do_traj_ik_default(lr, gripper_poses):
    gripper_xyzs, gripper_quats = [], []
    for pose in gripper_poses:
        xyz, quat = conversions.pose_to_trans_rot(pose)
        gripper_xyzs.append(xyz)
        gripper_quats.append(quat)
    manip = get_manipulator(lr)
    joint_positions, inds = trajectories.make_joint_traj(gripper_xyzs, gripper_quats, manip, "base_footprint", "%s_gripper_tool_frame"%lr, filter_options = 1+18)
    return joint_positions

# do ik by calling the plan_traj service
def do_traj_ik_opt(lr, gripper_poses):
    plan_traj_req = PlanTrajRequest()
    plan_traj_req.manip = "rightarm" if lr == 'r' else "leftarm"
    plan_traj_req.link = "%s_gripper_tool_frame"%lr
    plan_traj_req.task = "follow_cart"

    #set table bounds
    table_bounds = map(float, rospy.get_param("table_bounds").split())
    xmin, xmax, ymin, ymax, zmin, zmax = table_bounds
    plan_traj_req.xmlstring = \
    """
    <Environment>
        <KinBody name="%s">
            <Body type="static">
            <Geom type="box">
            <Translation> %f %f %f </Translation>
            <extents> %f %f %f </extents>
            </Geom>
            </Body>
        </KinBody>
    </Environment>
    """ \
    % ("table", 
      (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2,
      (xmax-xmin)/2, (ymax-ymin)/2, (zmax-zmin)/2)

    plan_traj_req.robot_joints = Globals.pr2.robot.GetDOFValues()

    pose_vals = []
    for pose in gripper_poses:
        xyz, quat = conversions.pose_to_trans_rot(pose)
        pose_vals.append(np.concatenate((quat, xyz)))
    plan_traj_req.goal = np.array(pose_vals)[::5].flatten()

    plan_traj_service_name = "plan_traj"
    rospy.wait_for_service(plan_traj_service_name)
    plan_traj_service_proxy = rospy.ServiceProxy(plan_traj_service_name, PlanTraj)
    try:
        plan_traj_resp = plan_traj_service_proxy(plan_traj_req)
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
        return np.array([])

    joint_positions = np.array(plan_traj_resp.trajectory).reshape((-1, 7))
    return joint_positions

def plot_gripper_xyzs_from_poses(lr, gripper_poses):
    manip = get_manipulator(lr)
    gripper_xyzs, gripper_quats = [],[]
    for pose in gripper_poses:
        xyz, quat = conversions.pose_to_trans_rot(pose)
        gripper_xyzs.append(xyz)
        gripper_quats.append(quat)
    make_verb_traj.plot_curve(gripper_xyzs, (1, 1, 0, 1))

def exec_traj(req, traj_ik_func=do_traj_ik_default):
    assert isinstance(req, ExecTrajectoryRequest)
    del Globals.handles[1:]
    
    traj = req.traj

    body_traj = {}
    for (lr,gripper_poses, gripper_angles) in zip("lr",[traj.l_gripper_poses.poses,traj.r_gripper_poses.poses], [traj.l_gripper_angles,traj.r_gripper_angles]):
        if len(gripper_poses) == 0: continue
        gripper_angles = np.array(gripper_angles)
        rospy.logwarn("warning! gripper angle hacks")
        gripper_angles[gripper_angles < .04] = gripper_angles[gripper_angles < .04] - .02

        gripper_angles = get_proper_gripper_angles(lr, gripper_angles)

        plot_gripper_xyzs_from_poses(lr, gripper_poses)

        #do ik
        joint_positions = traj_ik_func(lr, gripper_poses)
        if len(joint_positions) == 0:
            return ExecTrajectoryResponse(success=False)
        joint_positions = ku.smooth_positions(joint_positions, .15)
        
        body_traj["%s_arm"%lr] = joint_positions
        body_traj["%s_gripper"%lr] = gripper_angles

        pose_array = gm.PoseArray()
        pose_array.header.frame_id = "base_footprint"
        pose_array.header.stamp = rospy.Time.now()
        pose_array.poses = traj.l_gripper_poses.poses if lr == 'l' else traj.r_gripper_poses.poses
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1)))

    yn = yes_or_no("continue?")
    if yn:
        lt.go_to_start(Globals.pr2, body_traj)
        lt.follow_trajectory_with_grabs(Globals.pr2, body_traj)

        Globals.pr2.join_all()
        return ExecTrajectoryResponse(success=True)
    else:
        return ExecTrajectoryResponse(success=False)
    
