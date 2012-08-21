import h5py
import lfd
import os.path as osp
import lfd.lfd_traj as lt
from brett2.PR2 import PR2
import numpy as np
import rospy
if rospy.get_name() == "/unnamed": rospy.init_node("exec_demo")
pr2 = PR2.create()
F = h5py.File(osp.join(osp.dirname(lfd.__file__),"data/knot_segs.h5"))
demo = F["knot00.00"]

jpos = np.asarray(demo["joint_states"]["position"])
jname = list(demo["joint_states"]["name"])

r_arm_inds = [jname.index(name) for name in pr2.rarm.joint_names]
l_arm_inds = [jname.index(name) for name in pr2.larm.joint_names]
r_grip_inds= [jname.index(name) for name in pr2.rgrip.joint_names]
l_grip_inds= [jname.index(name) for name in pr2.lgrip.joint_names]

l_grab = jpos[:,l_grip_inds] < .07
r_grab = jpos[:,r_grip_inds] < .07

j_l_grip = jpos[:,l_grip_inds]
j_l_grip[l_grab] = 0
j_l_grip = np.fmin(j_l_grip, .08)

j_r_grip = jpos[:,r_grip_inds]
j_r_grip[r_grab] = 0
j_r_grip = np.fmin(j_r_grip, .08)

lt.follow_trajectory_with_grabs(pr2, {"l_gripper":j_l_grip,
                                      "r_gripper":j_r_grip,
                                      "l_arm":jpos[:,l_arm_inds],
                                      "r_arm":jpos[:,r_arm_inds]},
                                l_grab, r_grab)