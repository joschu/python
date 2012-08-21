import argparse
parser = argparse.ArgumentParser()
parser.add_argument("joints",type=str, nargs="+")
args = parser.parse_args()

import trajectory_msgs.msg as tm
import sensor_msgs.msg as sm
import rospy
import matplotlib.pyplot as plt
import numpy as np


joint2times_cmd = {}
joint2vals_cmd = {}
joint2times_act = {}
joint2vals_act = {}

DONE = False

for joint in args.joints:
    joint2times_cmd[joint] = []
    joint2vals_cmd[joint] = []
    joint2times_act[joint] = []
    joint2vals_act[joint] = []

def cmd_callback(msg):
    if DONE: return
    arr = np.zeros((len(msg.points), len(msg.joint_names)))
    times = np.zeros(len(msg.points))
    for (i,jtp) in enumerate(msg.points):
        arr[i] = jtp.positions
        times[i] = jtp.time_from_start.to_sec()
        
    times += msg.header.stamp.to_sec()
    
    for joint_name in args.joints:
        if joint_name in msg.joint_names:
            i = msg.joint_names.index(joint_name)
            joint2vals_cmd[joint_name].append(arr[:,i])
            joint2times_cmd[joint_name].append(times)
            
    
joint_state_inds = []
def act_callback(msg):
    if DONE: return
    if len(joint_state_inds) == 0:
        for joint_name in args.joints:
            joint_state_inds.append(msg.name.index(joint_name))
            
    for (i,joint_name) in zip(joint_state_inds,args.joints):
        joint2vals_act[joint_name].append(msg.position[i])
        joint2times_act[joint_name].append(msg.header.stamp.to_sec())
        

rospy.init_node("trajectory_diagnostic_plotter")
joint_sub = rospy.Subscriber("/joint_states", sm.JointState, act_callback)
r_arm_traj_sub = rospy.Subscriber("/r_arm_controller/command", tm.JointTrajectory, cmd_callback)
l_arm_traj_sub = rospy.Subscriber("/l_arm_controller/command", tm.JointTrajectory, cmd_callback)
r_gripper_traj_sub = rospy.Subscriber("/r_gripper_traj_diagnostic", tm.JointTrajectory, cmd_callback)
l_gripper_traj_sub = rospy.Subscriber("/l_gripper_traj_diagnostic",tm.JointTrajectory, cmd_callback)

raw_input("press enter when done")
DONE = True


colors = 'rgbcmyk'
offsets, scales = [],[]
for (i,joint) in enumerate(args.joints):
    vmin = np.min(np.r_[joint2vals_act[joint],np.concatenate(joint2vals_cmd[joint])])
    vmax = np.max(np.r_[joint2vals_act[joint],np.concatenate(joint2vals_cmd[joint])])

    offset = (vmin+vmax)/2
    scale = (vmax - vmin) + 1e-4
    offsets.append(offset)
    scales.append(scale)
    plt.plot(joint2times_act[joint], (joint2vals_act[joint]-offset)/scale,colors[i]+'-',lw=3)
plt.legend(args.joints)
for (i,joint) in enumerate(args.joints):
    for (times, vals) in zip(joint2times_cmd[joint],joint2vals_cmd[joint]):
        plt.plot(times, (vals-offsets[i])/scales[i],colors[i]+'x-',lw=1)
plt.show()