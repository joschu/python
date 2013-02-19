#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('joint_states_listener')
import rospy
import rosbag
from joint_states_listener.srv import ReturnJointStates
import sys
import numpy as np

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
 

## Record arm and gripper trajectories
if __name__ == "__main__":
    rospy.init_node('joint_states_trajectory_recorder')

    traj_file_path = '/home/mallory/fuerte_workspace/sandbox/suturing/pr2_suturing/joint_states_listener/trajectories'

    record_time = rospy.Duration.from_sec(300.0) #pt 1, 2
    #record_time = rospy.Duration.from_sec(240.0) #pt 3, 4, 5

    traj_larm = []
    traj_rarm = []
    traj_lgrip = []
    traj_rgrip = []  

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
    
    print 'record_time:', record_time.secs
    start_time = rospy.Time.now().secs

    while((rospy.Time.now().secs - start_time) < record_time.secs):

        if (rospy.Time.now().secs - start_time) % 5 == 0:
            print 'time left:', record_time.secs - (rospy.Time.now().secs - start_time)

        (position, velocity, effort) = call_return_joint_states(joint_names)

        #print "position:", pplist(position)
        #print "velocity:", pplist(velocity)
        #print "effort:  ", pplist(effort)
        
        traj_larm.append(position[0:7])
        traj_rarm.append(position[7:14])
        traj_lgrip.append(position[14])
        traj_rgrip.append(position[15])  

    print 'end_time:', rospy.Time.now().secs
    print 'finished recording'

    ## Save suture trajectories to file
    np.save(traj_file_path + '/full_running_suture/pt1_tj_larm.npy', np.array(traj_larm))
    np.save(traj_file_path + '/full_running_suture/pt1_tj_rarm.npy', np.array(traj_rarm))
    np.save(traj_file_path + '/full_running_suture/pt1_tj_lgrip.npy', np.array(traj_lgrip))
    np.save(traj_file_path + '/full_running_suture/pt1_tj_rgrip.npy', np.array(traj_rgrip))

## running suture:
## height: 3rd bolt from the bottom on the side of the bot is level with bottom of foam
## pt1: pierce both flaps, pull twice with right hand, pass needle to right hand, pull with left hand until through,
##      put needle in stand, end in 'side'
## pt2: left 2 loop knot (needle not perfectly in stand at end)
## pt3: right 1 loop knot
## pt4: left 1 loop knot
## pt5: right 1 loop knot
## pt6: needle through both flaps, pull twice with right hand, pass needle to right hand, pull twice with left hand,
##      put needle in stand, end in 'side'
## ...



