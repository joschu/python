import roslib;
roslib.load_manifest('rospy')
import rospy
import numpy as np
import utils.conversions as conv
import utils.math_utils as mu
import kinematics.kinematics_utils as ku

BodyState = np.dtype([
    ("r_arm", float, 7),
    ("l_arm", float, 7),
    ("r_gripper", float, 1),
    ("l_gripper", float, 1),
    ("head", float, 2),
    ("base", float, 3)
])

def make_joint_traj(xyzs, quats, arm, filter_options = 0):
    "do ik and then fill in the points where ik failed"

    n = len(xyzs)
    assert len(quats) == n
    print n
    
    robot = arm.GetRobot()
    joint_inds = arm.GetArmJoints()
    robot.SetActiveDOFs(joint_inds)
    orig_joint = robot.GetActiveDOFValues()
    
    joints = []
    inds = []

    for i in xrange(0,n):
        joint = arm.FindIKSolution(conv.trans_rot_to_hmat(xyzs[i], quats[i]), filter_options)
        print conv.trans_rot_to_hmat(xyzs[i], quats[i])
        if joint is not None: 
            joints.append(joint)
            inds.append(i)
            robot.SetActiveDOFValues(joint)


    robot.SetActiveDOFValues(orig_joint)
    
    
    joints = np.array(joints)
    for roll_ind in [2,4,6]:
        joints[:,roll_ind] = np.unwrap(np.r_[orig_joint[roll_ind], joints[:,roll_ind]])[1:]
    
    rospy.loginfo("found ik soln for %i of %i points",len(inds), n)

    joints2 = mu.interp2d(np.arange(n), inds, joints)
    return joints2


def follow_body_traj(pr2, traj, times, r_arm = False, l_arm = False, r_gripper = False, l_gripper = False, head = False, base = False):
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
        pr2.base.follow_timed_trajectory(times, traj["base"], frame_id = "/map")
    
        
        
    pr2.join_all()
