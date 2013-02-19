#import roslib;
#roslib.load_manifest('rospy')
import rospy
import numpy as np
from brett2 import PR2
import jds_utils.conversions as conv
import jds_utils.math_utils as mu
from kinematics import retiming
import scipy.interpolate as si

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
    joint_inds = manip.GetArmJoints()
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


def follow_body_traj(pr2, traj, times=None, r_arm = False, l_arm = False, r_gripper = False, l_gripper = False, head = False, base = False):

    assert isinstance(pr2, PR2.PR2)
    pr2.update_rave()

    # originally 0.2
    vel_mult = 0.4
    
    s = 0.3
    vel_l = []
    vel_r = []

    vel_limits = np.r_[pr2.rarm.vel_limits*vel_mult, pr2.larm.vel_limits*vel_mult, pr2.rgrip.vel_limits, pr2.lgrip.vel_limits]
    acc_limits = np.r_[pr2.rarm.acc_limits, pr2.larm.acc_limits, np.inf, np.inf]

    times, inds = retiming.retime(flatten_compound_dtype(traj), vel_limits, acc_limits)

    traj = traj[inds]
    smooth_traj = traj

    n = len(traj['r_arm'])

    (r_tck, _) = si.splprep(traj['r_arm'].T, s=s, u=times, k=3)
    smooth_traj['r_arm'] = np.r_[si.splev(times, r_tck, der=0)].T
    vel_r = np.r_[si.splev(times, r_tck, der=1)].T

    (l_tck, _) = si.splprep(traj['l_arm'].T, s=s, u=times, k=3)
    smooth_traj['l_arm'] = np.r_[si.splev(times, l_tck, der=0)].T
    vel_l = np.r_[si.splev(times, l_tck, der=1)].T

    # make sure gripper closes all the way
    for i in range(n):
        if smooth_traj['r_gripper'][i] < 0.005:
            smooth_traj['r_gripper'][i] = -0.01
        if smooth_traj['l_gripper'][i] < 0.005:
            smooth_traj['l_gripper'][i] = -0.01

    if r_arm:
        pr2.rarm.goto_joint_positions(smooth_traj['r_arm'][0])
    if l_arm:
        pr2.larm.goto_joint_positions(smooth_traj['l_arm'][0])
    if r_gripper:
        pr2.rgrip.set_angle(smooth_traj['r_gripper'][0])
    if l_gripper:
        pr2.lgrip.set_angle(smooth_traj['l_gripper'][0])
    if head or base:
        raise NotImplementedError
    
    pr2.join_all()
    
    rospy.sleep(5.0) 

    if r_gripper:
        pr2.rgrip.follow_timed_trajectory(times, smooth_traj['r_gripper'])
    if l_gripper:
        pr2.lgrip.follow_timed_trajectory(times, smooth_traj['l_gripper'])

    rospy.sleep(0.5)

    if r_arm:
        pr2.rarm.follow_timed_joint_trajectory(smooth_traj['r_arm'], vel_r, times)
    if l_arm:
        pr2.larm.follow_timed_joint_trajectory(smooth_traj['l_arm'], vel_l, times)
    if head:
        pr2.head.follow_timed_trajectory(times, traj['head'])
    if base:
        pr2.base.follow_timed_trajectory(times, traj['base'], frame_id = '/map')       
        
    pr2.join_all()

