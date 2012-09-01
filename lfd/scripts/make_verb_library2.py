#!/usr/bin/env python

import lfd
from lfd import bag_proc
import rosbag
import os.path as osp
import os
import yaml
import h5py
from copy import copy
from lfd import verbs
from lfd.utils_lfd import group_to_dict

link_names = ["r_gripper_tool_frame", "l_gripper_tool_frame"]

data_dir = osp.join(osp.dirname(lfd.__file__), "data")
h5path = osp.join(data_dir, "verbs2.h5")
if os.path.exists(h5path): os.remove(h5path)
verb_lib = h5py.File(h5path,"w")

for (verb_name, verb_info) in verbs.get_all_demo_info().items():
    bag = rosbag.Bag(osp.join(data_dir, verb_info["bag_file"]))
    
    segs = bag_proc.create_segment_without_look(bag, link_names)    
    if len(segs) > 1: print "warning: more than one segment found"
    kinematics_data = segs[0]
    
    verb_data = copy(kinematics_data)
    seg_file = h5py.File(osp.join(data_dir,verb_info["seg_file"]),"r")
    
    bag_proc.dict_to_hdf(verb_lib, verb_data, verb_name)
    seg_file.copy("/", verb_lib[verb_name],"object_clouds")
    verb_lib[verb_name]["arms_used"] = verb_info["arms_used"]    
    