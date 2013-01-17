#import roslib;
#roslib.load_manifest('rospy')
import rospy
import numpy as np
from brett2 import PR2
import jds_utils.conversions as conv
import jds_utils.math_utils as mu
import kinematics.kinematics_utils as ku
from kinematics import retiming

BodyState = np.dtype([
    ("r_arm", float, 7),
    ("l_arm", float, 7),
    ("r_gripper", float, 1),
    ("l_gripper", float, 1),
    ("head", float, 2),
    ("base", float, 3)
])

def make_joint_traj(xyzs, quats, manip, ref_frame, targ_frame, filter_options = 0):
    "do ik and then fill in the points where ik failed"

    n = len(xyzs)
    assert len(quats) == n
    
    robot = manip.GetRobot()
    joint_inds = manip.GetArmIndices()
    robot.SetActiveDOFs(joint_inds)
    orig_joint = robot.GetActiveDOFValues()
    
    joints = []
    inds = []

    for i in xrange(0,n):
        mat4 = conv.trans_rot_to_hmat(xyzs[i], quats[i])
        joint = PR2.cart_to_joint(manip, mat4, ref_frame, targ_frame, filter_options)
        if joint is not None: 
            joints.append(joint)
            inds.append(i)
            robot.SetActiveDOFValues(joint)


    robot.SetActiveDOFValues(orig_joint)
    
    
    rospy.loginfo("found ik soln for %i of %i points",len(inds), n)
    if len(inds) > 2:
        joints2 = mu.interp2d(np.arange(n), inds, joints)
        return joints2, inds
    else:
        return np.zeros((n, len(joints))), []



def follow_body_traj2(pr2, bodypart2traj, times=None, wait=True, base_frame = "/base_footprint"):    

    rospy.loginfo("following trajectory with bodyparts %s", " ".join(bodypart2traj.keys()))
    trajectories = []
    vel_limits = []
    acc_limits = []
    bodypart2inds = {}
    
    n_dof = 0
    name2part = {"l_gripper":pr2.lgrip, 
                 "r_gripper":pr2.rgrip, 
                 "l_arm":pr2.larm, 
                 "r_arm":pr2.rarm,
                 "base":pr2.base}
    
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

    times = retiming.retime_with_vel_limits(trajectories, vel_limits)
    times_up = np.arange(0,times[-1]+1e-4,.1)
    traj_up = mu.interp2d(times_up, times, trajectories)
    
    for (name, part) in name2part.items():
        if name in bodypart2traj:
            part_traj = traj_up[:,bodypart2inds[name]]
            if name == "l_gripper" or name == "r_gripper":
                part.follow_timed_trajectory(times_up, part_traj.flatten())
            elif name == "l_arm" or name == "r_arm":
                vels = ku.get_velocities(part_traj, times_up, .001)
                part.follow_timed_joint_trajectory(part_traj, vels, times_up)
            elif name == "base":
                part.follow_timed_trajectory(times_up, part_traj, base_frame)
    if wait: pr2.join_all()    
    return True


def flatten_compound_dtype(compound_array):
    arrays = []
    for desc in compound_array.dtype.descr:
        field = desc[0]        
        arr = compound_array[field]
        if arr.ndim == 1:
            float_arr = arr[:,None].astype('float')
        elif arr.ndim == 2:
            float_arr = arr.astype('float')
        else:
            raise Exception("subarray with field %s must be 1d or 2d"%field)
        arrays.append(float_arr)
        
    return np.concatenate(arrays, axis=1)

def follow_body_traj(pr2, traj, times=None, r_arm = False, l_arm = False, r_gripper = False, l_gripper = False, head = False, base = False, 
                     base_frame = "/odom_combined", wait=True):
    assert isinstance(pr2, PR2.PR2)
    pr2.update_rave()
    if r_arm:
        pr2.rarm.goto_joint_positions(traj["r_arm"][0])
    if l_arm:
        pr2.larm.goto_joint_positions(traj["l_arm"][0])
    if r_gripper:
        pr2.rgrip.set_angle(traj["r_gripper"][0])
    if l_gripper:
        pr2.lgrip.set_angle(traj["l_gripper"][0])
    if head:
        raise NotImplementedError
    if base:
        print "warning: not going to initial base position (goto position not implemented)"
        #pr2.base.goto_pose(traj["base"][0], base_frame)
    pr2.join_all()
    
    if times is None:
        vel_limits = np.r_[pr2.rarm.vel_limits, pr2.larm.vel_limits, .02, .02, pr2.head.vel_limits*vel_mult, [.2,.2,.2]]
        acc_limits = np.r_[pr2.rarm.acc_limits, pr2.larm.acc_limits, np.inf, np.inf, pr2.head.acc_limits, [1,1,1]]
        #times, inds = retiming.retime(flatten_compound_dtype(traj), vel_limits, acc_limits)
        times, inds = retiming.retime2(flatten_compound_dtype(traj), vel_limits)
        traj = traj[inds]

    if r_arm:
        pr2.rarm.follow_timed_joint_trajectory(traj["r_arm"],ku.get_velocities(traj["r_arm"],times,.001),times)
    if l_arm:
        pr2.larm.follow_timed_joint_trajectory(traj["l_arm"],ku.get_velocities(traj["l_arm"],times,.001),times)
    if r_gripper:
        pr2.rgrip.follow_timed_trajectory(times, traj["r_gripper"])
    if l_gripper:
        pr2.lgrip.follow_timed_trajectory(times, traj["l_gripper"])
    if head:
        pr2.head.follow_timed_trajectory(times, traj["head"])
    if base:
        pr2.base.follow_timed_trajectory(times, traj["base"], frame_id = base_frame)
                    
    if wait: pr2.join_all()



def follow_rave_trajectory(pr2, ravetraj, dof_inds, use_base = False, base_frame="/base_footprint"):

    assert ravetraj.shape[1] == len(dof_inds) + 3*int(use_base)
    bodypart2traj = {}
    

    rave2ros = {}
    name2part = {"l_gripper":pr2.lgrip, 
                 "r_gripper":pr2.rgrip, 
                 "l_arm":pr2.larm, 
                 "r_arm":pr2.rarm}
    for (partname, part) in name2part.items():
        for (ijoint, jointname) in enumerate(part.joint_names):
            rave2ros[pr2.robot.GetJoint(jointname).GetDOFIndex()] = (partname, ijoint)
        bodypart2traj[partname] = np.repeat(np.asarray(part.get_joint_positions())[None,:], len(ravetraj), axis=0)

            
    
    bodypart2used = {}    
            
    for (ravecol, dof_ind) in enumerate(dof_inds):
        if dof_ind in rave2ros:
            partname, partind = rave2ros[dof_ind]
            bodypart2traj[partname][:,partind] = ravetraj[:,ravecol]
        elif dof_ind == pr2.robot.GetJoint("r_gripper_l_finger_joint").GetDOFIndex():
            partname, partind = "r_gripper", 0
            bodypart2traj[partname][:,partind] = ravetraj[:,ravecol]/5.81
        elif dof_ind == pr2.robot.GetJoint("l_gripper_l_finger_joint").GetDOFIndex():
            partname, partind = "l_gripper", 0
            bodypart2traj[partname][:,partind] = ravetraj[:,ravecol]/5.81
        else:
            jointname = pr2.robot.GetJointFromDOFIndex(dof_ind).GetName()
            raise Exception("I don't know how to control this joint %s"%jointname)
        bodypart2used[partname] = True
        
    for partname in list(bodypart2traj.keys()):
        if partname not in bodypart2used:
            del bodypart2traj[partname]            

    if use_base:
        base_traj = ravetraj[:,-3:]
        bodypart2traj["base"] = base_traj
        
    follow_body_traj2(pr2, bodypart2traj, base_frame = base_frame)
    