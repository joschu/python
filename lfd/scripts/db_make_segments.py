#!/usr/bin/env python
"""
Best on yaml file, take extended trajectory (in an hdf5 file) and break it up into trajectory segments, 
each with a corresponding point cloud
"""


import argparse
from os.path import exists,join,dirname

parser = argparse.ArgumentParser()
parser.add_argument("h5in")
parser.add_argument("h5out")
parser.add_argument("yamlfile",type=argparse.FileType("r"))
parser.add_argument("--kinect_frame", default="/openni_rgb_optical_frame")
args = parser.parse_args()
assert exists(args.h5in)

import h5py, rosbag, rospy
import itertools
import sensor_msgs.msg as sm
import numpy as np
from brett2.ros_utils import pc2xyzrgb, transform_points
import yaml
import traceback
from brett2 import mytf

h5in = h5py.File(args.h5in)
h5out = h5py.File(args.h5out,"r+" if exists(args.h5out) else "w")


yamlfile = yaml.load(args.yamlfile)
traj = h5in[yamlfile["traj_path"]]
bagfile = traj["bag_file"].value

bag = rosbag.Bag(bagfile)
seg_dicts = yamlfile["segments"]

listener = mytf.TransformListener()

t_start_bag = bag.read_messages().next()[2].to_sec()

cloud_times, cloud_xyzs, cloud_bgrs = [],[],[]
for (topic, msg, t) in bag.read_messages():
    if topic=="/tf":
        listener.callback(msg)
    elif hasattr(msg, "row_step"):

        xyz, bgr = pc2xyzrgb(msg)
        try:
            xyz_tf = transform_points(xyz, listener, traj["ref_frame"].value, args.kinect_frame)
            cloud_xyzs.append(xyz_tf)
            cloud_bgrs.append(bgr)  
            cloud_times.append(t.to_sec())
        except Exception:
            print "failed to transform points"
            traceback.print_exc()
cloud_times = np.array(cloud_times) 
cloud_times -= t_start_bag

traj_times = np.asarray(traj["times"])-t_start_bag
for (i_seg,seg_dict) in enumerate(seg_dicts):
    i_start, i_stop = np.searchsorted(traj_times, (seg_dict["start"],seg_dict["stop"]))
    seg_name = "%s.%.2i"%(traj.name,i_seg)
    if seg_name in h5out: del traj[seg_name]
    seg = h5out.create_group(seg_name)
    for field in ["joints", "times", 
                  "r_gripper_xyzs", "r_gripper_xyzs1", "r_gripper_xyzs2", "r_gripper_quats", "r_gripper_angles",
                  "l_gripper_xyzs", "l_gripper_xyzs1", "l_gripper_xyzs2","l_gripper_quats", "l_gripper_angles"]:
        seg[field] = traj[field][i_start:i_stop+1]
    i_cloud = np.abs(cloud_times - seg_dict["look"]).argmin()    
    seg["cloud_xyz"] = cloud_xyzs[i_cloud]
    seg["cloud_bgr"] = cloud_bgrs[i_cloud]
    seg["arms_used"] = seg_dict["arms_used"]
    
done = h5out.create_group("done_%s"%traj.name.strip('/'))
i_cloud = np.abs(cloud_times - yamlfile["done"]).argmin()
done["cloud_xyz"] = cloud_xyzs[i_cloud]
done["cloud_bgr"] = cloud_xyzs[i_cloud]
    
h5in.close()
h5out.close() 
    
    
    

