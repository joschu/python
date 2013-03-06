#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("task")
parser.add_argument("part_name")
args = parser.parse_args()


import lfd
from lfd import bag_proc as bp
import rosbag
import os.path as osp
from jds_utils.yes_or_no import yes_or_no
import os, sys
import yaml
import h5py

IROS_DATA_DIR = os.getenv("IROS_DATA_DIR")
task_file = osp.join(IROS_DATA_DIR, "suture_demos2.yaml")
with open(osp.join(IROS_DATA_DIR,task_file),"r") as fh:
    task_info = yaml.load(fh)


db_file = osp.join(IROS_DATA_DIR, task_info[args.task][args.part_name]["db_file"])
print 'Writing to', db_file
if osp.exists(db_file):
    if yes_or_no(db_file + ' already exists. Overwrite?'):
        os.remove(db_file)
    else:
        print 'Aborting.'
        sys.exit(1)

task_lib = h5py.File(db_file, mode="w")
link_names = ["r_gripper_tool_frame", "l_gripper_tool_frame"]

for (i_demo,bag_file) in enumerate(task_info[args.task][args.part_name]["demo_bag"]):
    bag = rosbag.Bag(bag_file)
    demo_segs = bp.create_segments(bag, link_names)
                        
    for (i_seg,demo_seg) in enumerate(demo_segs):
        #if demo_seg["done"]:
        #    path = "%.2i.done"%i_demo
        #else:
        path = "%.2i.%.2i"%(i_demo, i_seg)
        demo_seg["demo_index"] = i_demo
        demo_seg["seg_index"] = i_seg
        bp.dict_to_hdf(task_lib, demo_seg, path)
        
        
