#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("task")
args = parser.parse_args()


import lfd
from lfd import bag_proc
import rosbag
import os.path as osp
from jds_utils.yes_or_no import yes_or_no
import os, sys
import yaml
import h5py

data_dir = osp.join(osp.dirname(lfd.__file__), "data")
task_file = osp.join(data_dir, "knot_demos.yaml")
with open(osp.join(data_dir,task_file),"r") as fh:
    task_info = yaml.load(fh)


db_file = osp.join(data_dir, task_info[args.task]["db_file"])
print 'Writing to', db_file
if osp.exists(db_file):
    if yes_or_no(db_file + ' already exists. Overwrite?'):
        os.remove(db_file)
    else:
        print 'Aborting.'
        sys.exit(1)
task_lib = h5py.File(db_file, mode="w")



for (i_demo,bag_file) in enumerate(task_info[args.task]["demo_bags"]):
    bag = rosbag.Bag(bag_file)
    demo_segs = bag_proc.create_segments(bag, ["r_gripper_tool_frame", "r_gripper_l_finger_tip_link", "r_gripper_r_finger_tip_link", 
                                               "l_gripper_tool_frame", "l_gripper_l_finger_tip_link", "l_gripper_r_finger_tip_link"])
                        
    for (i_seg,demo_seg) in enumerate(demo_segs):
        if demo_seg["done"]:
            path = "%.2i.done"%i_demo
        else:
            path = "%.2i.%.2i"%(i_demo, i_seg)
        demo_seg["demo_index"] = i_demo
        demo_seg["seg_index"] = i_seg
        bag_proc.dict_to_hdf(task_lib, demo_seg, path)
