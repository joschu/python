import openravepy
from utils.func_utils import once
from utils.math_utils import normalize
import openravepy
import numpy as np
import utils.conversions as conv
from brett2.PR2 import PR2
import rospy

def make_basis(a,b):
    p = normalize(a)
    q = normalize(b - np.dot(p,b)*p)
    r = np.cross(p,q)
    return c_[p,q,r]
    
def calc_hand_pose(lpos, rpos, ori):

    ynew0 = normalize(lpos - rpos)
    xnew0 = normalize(ori[:,0] - np.dot(ynew0, ori[:,0]) * ynew0)
    znew0 = np.cross(xnew0, ynew0)
    tool_ori = np.c_[xnew0, ynew0, znew0]
    tool_pos = (lpos + rpos)/2
    new_pose = np.eye(4)
    new_pose[:3,:3] = tool_ori
    new_pose[:3,3] = tool_pos
    new_open = np.linalg.norm(lpos - rpos) - 0.030284 # distance between frames when gripper closed
    return new_pose, new_open
    
    
if rospy.get_name() == "/unnamed": rospy.init_node("test_fingertip_ik",disable_signals=True)
pr2 = PR2.create()
pr2.update_rave()
robot = pr2.robot

pose = np.array([[  3.797e-01,   5.407e-04,  -9.251e-01,   5.102e-01],
       [  3.714e-03,   1.000e+00,   2.109e-03,  -1.868e-01],
       [  9.251e-01,  -4.236e-03,   3.797e-01,   1.096e+00],
       [  0.000e+00,   0.000e+00,   0.000e+00,   1.000e+00]])
pr2.rarm.goto_pose_matrix(pose, "base_footprint", "r_gripper_tool_frame", 18)
pr2.rgrip.set_angle(.05)
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