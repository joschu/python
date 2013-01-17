import argparse
parser = argparse.ArgumentParser()


import rospy
from time import time, sleep
import numpy as np
import sensor_msgs.msg as sm
import roslib
roslib.load_manifest("pr2_controllers_msgs"); 
import pr2_controllers_msgs.msg as pcm
import matplotlib.pyplot as plt
from brett2 import trajectories
import jds_utils.math_utils as mu
from brett2.PR2 import PR2, mirror_arm_joints


JOINT_NAME = "l_gripper_joint"
CMD_TOPIC = "/l_gripper_controller/command"
if rospy.get_name() == "/unnamed":
    rospy.init_node("check_command_vs_actual", disable_signals = True)

pr2 = PR2.create()
pr2.update_rave()
n_steps = 30
cmd_times = np.linspace(0,3,n_steps)
cmd_states = np.zeros(n_steps, dtype=trajectories.BodyState)

r_arm_cur = pr2.rarm.get_joint_positions()
cmd_states["r_arm"] = mu.linspace2d(r_arm_cur, mirror_arm_joints(pr2.rarm.L_POSTURES["untucked"]) - r_arm_cur,n_steps)
l_arm_cur = pr2.larm.get_joint_positions()
cmd_states["l_arm"] = mu.linspace2d(l_arm_cur, pr2.rarm.L_POSTURES["tucked"] - l_arm_cur, n_steps)
r_grip_cur = pr2.rgrip.get_angle()
cmd_states["r_gripper"] = np.linspace(r_grip_cur, .08 - r_grip_cur, n_steps) 
l_grip_cur = pr2.lgrip.get_angle()
cmd_states["l_gripper"] = np.linspace(l_grip_cur, .08 - l_grip_cur, n_steps) 
base_cur = pr2.base.get_pose("/odom_combined")
cmd_states["base"] = mu.linspace2d(base_cur, np.random.randn(3), n_steps)


trajectories.follow_body_traj(pr2, cmd_states, times=cmd_times, r_arm=True, l_arm=True, r_gripper=True, l_gripper=True, base=True, wait=False)


actual_times = []
actual_states = np.zeros(5000, trajectories.BodyState)
t_start = time()
size=0
while time() - t_start < cmd_times[-1]+1: #why doesn't is_moving() work???
    pr2.update_rave()
    actual_times.append(time() - t_start)
    
    actual_states["r_arm"][size] = pr2.rarm.get_joint_positions()
    actual_states["l_arm"][size] = pr2.larm.get_joint_positions()
    actual_states["r_gripper"][size] = pr2.rgrip.get_angle()
    actual_states["l_gripper"][size] = pr2.lgrip.get_angle()
    actual_states["base"][size] = pr2.base.get_pose("/odom_combined")
    actual_states[size]
    size += 1
    sleep(.01)
actual_states = actual_states[:size]    


plt.close('all')
for part in ("r_arm","l_arm","r_gripper","l_gripper","base"):
    plt.figure()
    plt.title(part)
    plt.plot(cmd_times, cmd_states[part],'b--')
    plt.plot(actual_times, actual_states[part],'b-')
plt.show()