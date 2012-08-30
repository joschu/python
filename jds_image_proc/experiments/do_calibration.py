#!/usr/bin/env python

import argparse, dir_tools
parser = argparse.ArgumentParser()
parser.add_argument("outdir")

args = parser.parse_args()
dir_tools.mkdir_ask(args.outdir)


import roslib
roslib.load_manifest('brett_pr2')
roslib.load_manifest('brett_utils')
import rospy
from brett_pr2.pr2 import PR2
from brett_utils.msg_utils import image2numpy
from brett_utils.shell_hax import rospack_find
import numpy as np
from time import sleep, time
import freenect
import sensor_msgs.msg as sm
from os.path import join
import cPickle
import cv2

rospy.init_node('record_calibration_images',disable_signals = True)

joints = np.loadtxt("calibration_joints.txt")

BRETT = PR2()
BRETT.rgrip.close(max_effort=-1)
BRETT.head.set_pan_tilt(0, .25)
BRETT.rarm.speed_limits /= 3
sleep(1)
        
info_left = rospy.wait_for_message('wide_stereo/left/camera_info', sm.CameraInfo)            
BRETT.rarm.goto_joint(joints[0])
bgr_lefts, bgr_kinects, depth_kinects, actual_joints = [],[],[], []

for (i,joint) in enumerate(joints):
    print "%i/%i"%(i,len(joints))
    BRETT.rarm.goto_joint(joint)
    sleep(.25)
    bgr_lefts.append(image2numpy(rospy.wait_for_message('wide_stereo/left/image_rect',sm.Image)))
    bgr_kinects.append(freenect.sync_get_video()[0][:,:,::-1].copy())
    depth_kinects.append(freenect.sync_get_depth()[0])
    actual_joints.append(rospy.wait_for_message('joint_states',sm.JointState).position)
    cv2.imshow('kinect',bgr_kinects[-1])
    cv2.imshow('left',bgr_lefts[-1])
    cv2.waitKey(10)

def npsave(basename, obj):    
    print "saving", basename, "...",
    np.save(join(args.outdir,basename),obj)
    print "OK"
    
npsave("bgr_lefts",bgr_lefts)
npsave("bgr_kinects",bgr_kinects)
npsave("depth_kinects",depth_kinects)
npsave("joints",joints)
npsave("actual_joints",actual_joints)
with open(join(args.outdir,"info_left.pkl"),"w") as fh: cPickle.dump(info_left, fh)