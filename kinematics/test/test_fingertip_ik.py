import openravepy
from utils.func_utils import once
from utils.math_utils import normalize
import openravepy
import numpy as np
import utils.conversions as conv
from brett2.PR2 import PR2
import rospy
from lfd.warping import calc_hand_pose

def make_basis(a,b):
    p = normalize(a)
    q = normalize(b - np.dot(p,b)*p)
    r = np.cross(p,q)
    return c_[p,q,r]
    
    
if rospy.get_name() == "/unnamed": rospy.init_node("test_fingertip_ik",disable_signals=True)
pr2 = PR2.create()
pr2.update_rave()
robot = pr2.robot

pose = np.array([[ 0.354,  0.924, -0.144,  0.63 ],
       [ 0.885, -0.381, -0.267,  0.091],
       [-0.301, -0.033, -0.953,  0.954],
       [ 0.   ,  0.   ,  0.   ,  1.   ]])

pr2.rarm.goto_pose_matrix(pose, "base_footprint", "r_gripper_tool_frame", 18)
#pr2.rgrip.set_angle(.05)
pr2.join_all()
pr2.update_rave()

lpos = robot.GetLink("r_gripper_l_finger_tip_link").GetTransform()[:3,3]
rpos = robot.GetLink("r_gripper_r_finger_tip_link").GetTransform()[:3,3]
ori = robot.GetLink("r_gripper_tool_frame").GetTransform()[:3,:3]

pose0 = robot.GetLink("r_gripper_tool_frame").GetTransform()
open0 = robot.GetJoint("r_gripper_joint").GetValues()[0]
pose1,open1 = calc_hand_pose(lpos, rpos, ori)


print pose0, open0
print pose1, open1