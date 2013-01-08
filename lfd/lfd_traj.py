"""
Some functions for generating trajectories.
Also see brett2.trajectories.
But these functions are slightly adapted for lfd stuff
"""


import numpy as np
from kinematics import retiming, kinematics_utils
import rospy
from time import time
from brett2 import PR2
import jds_utils.conversions as conv
import jds_utils.math_utils as mu
from jds_utils.math_utils import interp2d
import traj_ik_graph_search

ALWAYS_FAKE_SUCESS = False
USE_PLANNING = False

def merge_nearby_grabs(inds_sides):
    """
    currently this function isn't implemented properly
    here's what it's supposed to do: if you have an 'r' and an 'l' grab that are nearby in time
    merge them into a 'b' grab at the same time
    because it looks stupid when the robot grabs with one arm and then the other
    (TODO)
    """
    if len(inds_sides) < 2: 
        return inds_sides
    elif len(inds_sides) == 2:
        return [(inds_sides[1][0],'b')]
    else:
        raise NotImplementedError

def follow_trajectory_with_grabs(pr2, bodypart2traj):
    """
    bodypart2traj is a dictionary with zero or more of the following fields: {l/r}_grab, {l/r}_gripper, {l/r_arm}
    We'll follow all of these bodypart trajectories simultaneously
    Also, if the trajectory involves grabbing with the gripper, and the grab fails, the trajectory will abort
    """
    T = len(bodypart2traj.values()[0])
    l_grab = bodypart2traj["l_grab"] if "l_grab" in bodypart2traj else np.zeros(T,bool)
    r_grab = bodypart2traj["r_grab"] if "r_grab" in bodypart2traj else np.zeros(T,bool)
    l_after_grab_inds = np.flatnonzero(l_grab[1:] > l_grab[:-1])+1
    r_after_grab_inds = np.flatnonzero(r_grab[1:] > r_grab[:-1])+1
    inds_sides = sorted([(i,'l') for i in l_after_grab_inds] + [(i,'r') for i in r_after_grab_inds])
    inds_sides = merge_nearby_grabs(inds_sides)
    
    num_grabs = len(inds_sides)
    print "breaking trajectory into %i segments"%(num_grabs+1)
    go_to_start(pr2, bodypart2traj)
    i_begin = 0
    for (i_grab, side) in inds_sides:
        t_start = rospy.Time.now().to_sec()
        success = follow_trajectory(pr2, slice_traj(bodypart2traj, i_begin, i_grab))
        rospy.loginfo("follow traj result: %s, duration: %.2f", success, rospy.Time.now().to_sec() - t_start)
        if not success: return False
        t_start = rospy.Time.now().to_sec()
        success = close_gripper(pr2, side)
        rospy.loginfo("close gripper result: %s, duration: %.2f", success, rospy.Time.now().to_sec() - t_start)        
        if not success: return False        
        i_begin = i_grab
    t_start = rospy.Time.now().to_sec()
    success = follow_trajectory(pr2, slice_traj(bodypart2traj, i_begin, -1))
    rospy.loginfo("follow traj result: %s, duration: %.2f", success, rospy.Time.now().to_sec() - t_start)
    if not success: return False
    return True

def slice_traj(bodypart2traj, start, stop):
    """slice each array between start and stop index"""    
    out = {}
    for (bodypart, traj) in bodypart2traj.items():
        out[bodypart] = traj[start:stop]
    return out


def go_to_start(pr2, bodypart2traj):
    """go to start position of each trajectory in bodypart2traj"""
    name2part = {"l_gripper":pr2.lgrip, 
                 "r_gripper":pr2.rgrip, 
                 "l_arm":pr2.larm, 
                 "r_arm":pr2.rarm}
    for (name, part) in name2part.items():
        if name in bodypart2traj:
            if name == "l_gripper" or name == "r_gripper":
                part.set_angle_target(bodypart2traj[name][0])
            elif name == "l_arm" or name == "r_arm":
                if USE_PLANNING:
                    part.goto_joint_positions_planned(bodypart2traj[name][0])
                else:
                    part.goto_joint_positions(bodypart2traj[name][0])

    pr2.join_all()
    
    
def follow_trajectory(pr2, bodypart2traj):    
    """
    bodypart2traj is a dictionary with zero or more of the following fields: {l/r}_grab, {l/r}_gripper, {l/r_arm}
    We'll follow all of these bodypart trajectories simultaneously
    Also, if the trajectory involves grabbing with the gripper, and the grab fails, the trajectory will abort
    """        
    rospy.loginfo("following trajectory with bodyparts %s", " ".join(bodypart2traj.keys()))
    trajectories = []
    vel_limits = []
    acc_limits = []
    bodypart2inds = {}
    
    n_dof = 0
    name2part = {"l_gripper":pr2.lgrip, 
                 "r_gripper":pr2.rgrip, 
                 "l_arm":pr2.larm, 
                 "r_arm":pr2.rarm}
    
    for (name, part) in name2part.items():
        if name in bodypart2traj:
            traj = bodypart2traj[name]
            if traj.ndim == 1: traj = traj.reshape(-1,1)
            trajectories.append(traj)
            vel_limits.extend(part.vel_limits)
            acc_limits.extend(part.acc_limits)
            bodypart2inds[name] = range(n_dof, n_dof+part.n_joints)
            n_dof += part.n_joints
                        
    trajectories = np.concatenate(trajectories, 1)
    
    vel_limits = np.array(vel_limits)
    acc_limits = np.array(acc_limits)

    times = retiming.retime_with_vel_limits(trajectories, vel_limits/2)
    times_up = np.arange(0,times[-1],.1)
    traj_up = interp2d(times_up, times, trajectories)
    
    for (name, part) in name2part.items():
        if name in bodypart2traj:
            part_traj = traj_up[:,bodypart2inds[name]]
            if name == "l_gripper" or name == "r_gripper":
                part.follow_timed_trajectory(times_up, part_traj.flatten())
            elif name == "l_arm" or name == "r_arm":
                vels = kinematics_utils.get_velocities(part_traj, times_up, .001)
                part.follow_timed_joint_trajectory(part_traj, vels, times_up)
    pr2.join_all()    
    return True
    
def close_gripper(pr2, side):
    """
    return True if it's grabbing an object
    False otherwise
    """
    
    grippers = {'l':[pr2.lgrip], 'r':[pr2.rgrip], 'b':[pr2.lgrip, pr2.rgrip]}[side]
    
    success = True
    for gripper in grippers:
        gripper.close()
    
    pr2.join_all()
    rospy.sleep(.15)
    for gripper in grippers:
        if gripper.is_closed():
            rospy.logwarn("%s gripper grabbed air", side)
            success = False
    return success or ALWAYS_FAKE_SUCESS    

def make_joint_traj_by_graph_search(xyzs, quats, manip, targ_frame):
    #hmats = [conv.pose_to_hmat(pose) for pose in gripper_poses]
    assert(len(xyzs) == len(quats))
    hmats = [conv.trans_rot_to_hmat(xyz, quat) for xyz, quat in zip(xyzs, quats)]

    def ikfunc(hmat):
        return traj_ik_graph_search.ik_for_link(hmat, manip, targ_frame, return_all_solns=True)

    #manip = get_manipulator(lr)
    start_joints = manip.GetRobot().GetDOFValues(manip.GetArmIndices())
    #start_joints = Globals.pr2.robot.GetDOFValues(manip.GetArmIndices())

    def nodecost(joints):
        robot = manip.GetRobot()
        cost = 0
        with robot:
            robot.SetDOFValues(joints, manip.GetArmIndices())
            cost = 100*robot.GetEnv().CheckCollision(robot)
        return cost

    paths, costs, timesteps = traj_ik_graph_search.traj_cart2joint(hmats, ikfunc, start_joints=start_joints, nodecost=nodecost)

    i_best = np.argmin(costs)
    print "lowest cost of initial trajs:", costs[i_best]
    path_init = paths[i_best]
    return path_init, timesteps

def make_joint_traj(xyzs, quats, joint_seeds,manip, ref_frame, targ_frame,filter_options):
    """
    do ik to make a joint trajectory
    joint_seeds are the joint angles that this function will try to be close to
    I put that in so you could try to stay near the demonstration joint angles
    in retrospect, using that argument might be bad idea because it'll make the trajectory much more jerky in some situations
    (TODO)
    """
    n = len(xyzs)
    assert len(quats) == n
    
    robot = manip.GetRobot()
    robot.SetActiveManipulator(manip.GetName())
    joint_inds = manip.GetArmIndices()
    #robot.SetActiveDOFs(joint_inds)
    #orig_joint = robot.GetActiveDOFValues()
    orig_joint = robot.GetDOFValues(joint_inds)

    joints = []
    inds = []

    for i in xrange(0,n):
        mat4 = conv.trans_rot_to_hmat(xyzs[i], quats[i])
        #robot.SetActiveDOFValues(joint_seeds[i])
        robot.SetDOFValues(joint_seeds[i], joint_inds)
        joint = PR2.cart_to_joint(manip, mat4, ref_frame, targ_frame, filter_options)
        if joint is not None: 
            joints.append(joint)
            inds.append(i)
            #robot.SetActiveDOFValues(joint)
            robot.SetDOFValues(joint, joint_inds)


    #robot.SetActiveDOFValues(orig_joint)
    robot.SetDOFValues(orig_joint, joint_inds)
    
    
    rospy.loginfo("found ik soln for %i of %i points",len(inds), n)
    if len(inds) > 2:
        joints2 = mu.interp2d(np.arange(n), inds, joints)
        return joints2, inds
    else:
        return np.zeros((n, len(joints))), []
