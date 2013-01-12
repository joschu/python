#!/usr/bin/env python

import sys
import lfd
from lfd import bag_proc
import rosbag
import os.path as osp
import os
import yaml
import h5py
from copy import copy
from lfd import verbs, multi_item_verbs
from lfd.utils_lfd import group_to_dict
from jds_utils.colorize import colorize

link_names = ["r_gripper_tool_frame", "l_gripper_tool_frame"]

data_dir = osp.join(osp.dirname(lfd.__file__), "data")
h5path = osp.join(data_dir, "verbs2.h5")
if os.path.exists(h5path): os.remove(h5path)
verb_lib = h5py.File(h5path,"w")

verb_data_accessor = multi_item_verbs.VerbDataAccessor()

def make_verb_library_single():
    for (verb_name, verb_info) in verbs.get_all_demo_info().items():
        print colorize("processing demo: %s"%verb_name, "red")
        bag = rosbag.Bag(osp.join(data_dir, verb_info["bag_file"]))
        
        segs = bag_proc.create_segment_without_look(bag, link_names)    
        if len(segs) > 1: print "warning: more than one segment found"
        kinematics_data = segs[0]
        
        verb_data = copy(kinematics_data)
        seg_file = h5py.File(osp.join(data_dir,verb_info["seg_file"]),"r")
        
        bag_proc.dict_to_hdf(verb_lib, verb_data, verb_name)

        seg_file.copy("/", verb_lib[verb_name],"object_clouds")
        verb_lib[verb_name]["arms_used"] = verb_info["arms_used"]

# make the verb library for a multiple stage_name action    
# instead of creating a single entry in the hdf5 file for <verb_name>-<item_name>, create an entry for each stage_name
def make_verb_library_multi():
    for (verb_name, verb_info) in verb_data_accessor.get_all_demo_info().items():
        print colorize("processing demo: %s"%verb_name, "red")
        for stage_num, stage_name in enumerate(verb_info["stages"]):
            bag_file_name = "bags/%s.bag" % (stage_name)
            seg_file_name = "images/%s.seg.h5" % (stage_name)

            bag = rosbag.Bag(osp.join(data_dir, bag_file_name))
            
            segs = bag_proc.create_segment_without_look(bag, link_names)
            if len(segs) > 1: print "warning: more than one segment found"
            kinematics_data = segs[0]
            
            stage_data = copy(kinematics_data)
            seg_file = h5py.File(osp.join(data_dir, seg_file_name), "r")
            
            bag_proc.dict_to_hdf(verb_lib, stage_data, stage_name)

            seg_file.copy("/", verb_lib[stage_name], "object_clouds")
            # is the following needed, since it is stored in the yaml file?
            verb_lib[stage_name]["arms_used"] = verb_info["arms_used"][stage_num]

if __name__ == "__main__":
    if len(sys.argv) == 1:
        make_verb_library_single()
    elif len(sys.argv) == 2 and sys.argv[1] == "multi":
        make_verb_library_multi()
