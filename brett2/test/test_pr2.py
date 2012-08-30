from brett2.PR2 import PR2
import numpy as np
import roslib; 
roslib.load_manifest('rospy'); import rospy
from jds_utils.testing import testme, test_all
np.set_printoptions(precision=2)
@testme
def test_trajectory():
    brett.larm.goto_posture('untucked')
    joint_traj = np.array([
        pr2.Arm.L_POSTURES["untucked"],
        pr2.Arm.L_POSTURES["tucked"],
        pr2.Arm.L_POSTURES["tucked"],
        pr2.Arm.L_POSTURES["up"]])
    brett.larm.follow_joint_traj(joint_traj)

@testme
def test_arm_ik():
    brett.larm.goto_posture('untucked')    
    brett.join_all()
    brett.update_rave()
    posemat = brett.larm.get_pose_matrix('base_footprint','l_gripper_tool_frame')
    #posemat = brett.larm.manip.GetEndEffectorTransform()
    brett.larm.manip.FindIKSolution(posemat, 0)

    print posemat
    brett.larm.goto_pose_matrix(posemat, 'base_footprint', 'l_gripper_tool_frame')

def test_arm_fk():
    roslib.load_manifest('tf'); import tf as rostf
    listener = rostf.TransformListener()
    rospy.sleep(.5)
    
    now = rospy.Time.now()
    listener.waitForTransform("base_link", "r_gripper_tool_frame", now,rospy.Duration(1))
    xyz, quat = listener.lookupTransform("base_link", "r_gripper_tool_frame",now)
    posemat = brett.larm.get_pose_matrix("base_link", "r_gripper_tool_frame")
    assert np.allclose(xyz, posemat[:3,3],atol = 1e-4)
@testme
def test_head():
    brett.head.set_pan_tilt(0,0)
@testme
def test_torso():
    brett.torso.go_down()
    brett.torso.go_up()
@testme
def test_grippers():
    brett.lgrip.close()
    brett.lgrip.open()
@testme
def test_base():
    brett.base.set_twist(0,1,.1)


if __name__ == "__main__":
    if rospy.get_name() == "/unnamed": # for ipython convenience
        rospy.init_node("test_pr2",disable_signals=True)
    
    brett = PR2.create()
    rospy.sleep(.5)
    
    #test_all()
    test_arm_ik()

#




#brett.larm.get_pose_matrix(
#posemat = brett.larm.get_pose_matrix('torso_lift_link','l_gripper_palm_link')
#brett.larm.goto_pose_matrix(posemat, 'torso_lift_link', 'l_gripper_palm_link')
