#!/usr/bin/env python

"""
Interactively choose a verb, and then choose the targets, with the command line.
"""
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--grippers_closed",action="store_true")
args = parser.parse_args()


import matplotlib
matplotlib.use('Qt4Agg')


import roslib
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import *
import rospy
from brett2.ros_utils import xyz2pc
import numpy as np
import h5py
import sensor_msgs.msg as sm
from brett2 import ros_utils, PR2
from point_clouds import tabletop
from utils.yes_or_no import yes_or_no
import scipy.spatial.distance as ssd
from utils.math_utils import norms
roslib.load_manifest("snazzy_msgs")
from snazzy_msgs.srv import *
import lfd.lfd_traj as lt
import yaml

def select_from_list(list):
    strlist = [str(item) for item in list]
    while True:
        print "choose from the following options:"
        print " ".join("(%s)"%item for item in strlist)
        resp = raw_input("?) ")
        if resp not in strlist:
            print "invalid response. try again."
        else:
            return list[strlist.index(resp)]

if rospy.get_name() == "/unnamed": 
    rospy.init_node("interactive_verbs",disable_signals=True)

pr2 = PR2.PR2.create()

make_svc = rospy.ServiceProxy("make_verb_traj",MakeTrajectory)
seg_svc = rospy.ServiceProxy("interactive_segmentation",ProcessCloud)
chomp_svc = rospy.ServiceProxy("elaborate_trajectory", TrajectoryObjects)


F=h5py.File("/home/joschu/python/lfd/data/verbs.h5","r")
with open("/home/joschu/python/lfd/data/tool_info.yaml","r") as fh:
    tool_info = yaml.load(fh)
with open("/home/joschu/python/lfd/data/verb_info.yaml","r") as fh:
    verb_info = yaml.load(fh)

def make_and_execute_verb_traj(verb, arg_clouds):
    
    if "expansion" in verb_info[verb]:
        prim_verbs = verb_info[verb]["expansion"]
        print "expanding into primitive actions: %s"%" ".join(prim_verbs)
        for i in xrange(len(prim_verbs)):
            exec_success = make_and_execute_verb_traj(prim_verbs[i], arg_clouds[i:i+1])
            if exec_success == False:
                rospy.logerr("stopping complex verb %s", verb)
                return False
    else:    
        if len(F[verb]["demos"]) == 0:
            rospy.logerr("verb %s has no demonstrations!", verb)
            return False
        
        make_req = MakeTrajectoryRequest()
        make_req.object_clouds = [xyz2pc(cloud, "base_footprint") for cloud in arg_clouds]
        make_req.verb = verb
        make_resp = make_svc.call(make_req)

        made_traj = make_resp.traj
        lr = made_traj.arms_used
        chomp_req = TrajectoryObjectsRequest()
        chomp_req.trajectory = made_traj.gripper0_poses
        chomp_req.weights = [.5 for _ in made_traj.gripper0_poses.poses]
        chomp_req.whicharm = {"l":"left_arm","r":"right_arm"}[lr]
        chomp_req.duration = 10
        chomp_req.object_clouds = make_req.object_clouds


        approved = yes_or_no("execute trajectory?")        
        if approved:
            while yes_or_no("try again?"):
                try:
                    chomp_resp = chomp_svc.call(chomp_req)
                    break
                except Exception:
                    import traceback
                    traceback.print_exc()
            joints = np.array([jtp.position for jtp in chomp_resp.joint_trajectory.points])
            lt.go_to_start(pr2, {"%s_arm"%lr:joints, "%s_gripper"%lr:made_traj.gripper0_angles})            
            lt.follow_trajectory(pr2, {"%s_arm"%lr:joints, "%s_gripper"%lr:made_traj.gripper0_angles})

    
verb_list = [verb for verb in verb_info if "hidden" not in verb_info[verb]]
while True:

    if not args.grippers_closed:
        pr2.rgrip.open()
        pr2.lgrip.open()
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')

    verb = select_from_list(verb_list + ["swap-tool"])
    
    
    if verb in verb_info:
        arg_names = F[verb]["arg_names"]
        print "arguments: %s"%" ".join(arg_names)
                
        pr2.join_all()
        
        
        pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)
        pc_tf = ros_utils.transformPointCloud2(pc, pr2.tf_listener, "base_footprint", pc.header.frame_id)                
        
        n_sel = F[verb]["nargs"].value    
        arg_clouds = []
        for i_sel in xrange(n_sel):
            pc_sel = seg_svc.call(ProcessCloudRequest(cloud_in = pc_tf)).cloud_out
            xyz, rgb = ros_utils.pc2xyzrgb(pc_sel)
            arg_clouds.append(xyz.reshape(-1,3))
            
        make_and_execute_verb_traj(verb, arg_clouds)   
        raw_input("press enter to continue")        
                                
    elif verb in ["swap-tool"]:
        
        l_switch_posture = np.array([  0.773,   0.595,   1.563,  -1.433,   0.478,  -0.862,  .864])
        
        lr = select_from_list(["left", "right"])
        if lr == 'l':
            arm, grip, jpos = pr2.larm, pr2.lgrip, l_switch_posture
        else:
            arm, grip, jpos = pr2.rarm, pr2.rgrip, PR2.mirror_arm_joints(l_switch_posture)
            
        arm.goto_joint_positions(jpos)
        grip.open()
        raw_input("press enter when ready")
        grip.close()
        pr2.join_all()
                
        
