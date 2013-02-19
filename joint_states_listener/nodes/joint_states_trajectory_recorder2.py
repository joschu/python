#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('joint_states_listener')
import rospy
import rosbag
from joint_states_listener.srv import ReturnJointStates
import sys
import numpy as np
import threading as th


def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


## Pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])
 

## Class to signal when to stop recording
class AskDone(th.Thread):

    def __init__(self):
        th.Thread.__init__(self)
        self.done  = False

    def run(self):
        y = raw_input('Press any key to stop recording')
        self.done = True


## Record arm and gripper trajectories
if __name__ == "__main__":
    rospy.init_node('joint_states_trajectory_recorder')

    traj_file_path = '/home/mallory/fuerte_workspace/sandbox/suturing/pr2_suturing/joint_states_listener/trajectories'
    traj_name      = '/full_interrupted_suture/pt12'

    traj_larm   = []
    traj_rarm   = []
    traj_lgrip  = []
    traj_rgrip  = []  

    joint_names = ["l_shoulder_pan_joint",
                   "l_shoulder_lift_joint",
                   "l_upper_arm_roll_joint",
                   "l_elbow_flex_joint",
                   "l_forearm_roll_joint",
                   "l_wrist_flex_joint",
                   "l_wrist_roll_joint",
                   "r_shoulder_pan_joint",
                   "r_shoulder_lift_joint",
                   "r_upper_arm_roll_joint",
                   "r_elbow_flex_joint",
                   "r_forearm_roll_joint",
                   "r_wrist_flex_joint",
                   "r_wrist_roll_joint",
                   "l_gripper_joint",
                   "r_gripper_joint"]

    while(1):
        if rospy.Time.now().secs > 0:
            break
        else:
            continue
    
    start_time = rospy.Time.now().secs
    print 'Start Time:', start_time

    print 'Recording ...'
    ask_done = AskDone()
    ask_done.start()

    while (not ask_done.done):

        (position, velocity, effort) = call_return_joint_states(joint_names)
        
        traj_larm.append(position[0:7])
        traj_rarm.append(position[7:14])
        traj_lgrip.append(position[14])
        traj_rgrip.append(position[15])  

    print 'End Time:', rospy.Time.now().secs
    print 'Recording Duration:', rospy.Time.now().secs - start_time

    ask_done.join()

    print 'Finished recording'

    ## Save trajectories to file
    np.save(traj_file_path + traj_name + '_tj_larm.npy' , np.array(traj_larm))
    np.save(traj_file_path + traj_name + '_tj_rarm.npy' , np.array(traj_rarm))
    np.save(traj_file_path + traj_name + '_tj_lgrip.npy', np.array(traj_lgrip))
    np.save(traj_file_path + traj_name + '_tj_rgrip.npy', np.array(traj_rgrip))

    print 'Trajectories saved to file'


