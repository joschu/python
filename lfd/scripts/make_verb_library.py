#!/usr/bin/env python

"""
Generate verbs.h5, which has all the verb trajectories
"""


import lfd
import os.path as osp
import os
import h5py
import yaml
import subprocess
from utils.colorize import colorize
import numpy as np

data_dir = osp.join(osp.dirname(lfd.__file__), "data")
with open(osp.join(data_dir,"verb_demos.yaml"),"r") as fh:
    verb_demos = yaml.load(fh)
with open(osp.join(data_dir,"verb_info.yaml"),"r") as fh:
    verb_info = yaml.load(fh)    

verb_lib_path = osp.join(data_dir, "verbs.h5")
if osp.exists(verb_lib_path): os.unlink(verb_lib_path)

verb_lib = h5py.File(verb_lib_path, mode="w")    
for (verb, info) in verb_info.items():
    verb_group = verb_lib.create_group(verb)
    verb_group["nargs"] = len(info["arg_names"])
    verb_group["arg_names"] = info["arg_names"]
    verb_group.create_group("tool_transform")
    if "tool_transform" in info:
        verb_group["tool_transform"]["origin"] = info["tool_transform"]["origin"]
        verb_group["tool_transform"]["rotation"] = info["tool_transform"]["rotation"]
    else:
        verb_group["tool_transform"]["origin"] = [0.,0.,0.]
        verb_group["tool_transform"]["rotation"] = [0., 0., 0., 1.]
        
    verb_group.create_group("demos")
verb_lib.close()

for demo_info in verb_demos:
    verb_lib = h5py.File(verb_lib_path, mode="r+")    
    verb = demo_info["verb"]
    verb_group = verb_lib[verb]
    demo_num = len(verb_lib[verb]["demos"])
    demo_name = "traj%.2i"%demo_num
    demo_path = "%s/demos/%i"%(verb, demo_num)
    verb_lib.close()
    cmd = " ".join(["python","db_from_bag.py",osp.join(data_dir,demo_info["bag_file"]),verb_lib_path, demo_path, demo_info["arms_used"]])
    print colorize(cmd,"red")
    subprocess.check_call(cmd, shell=True)
    verb_lib = h5py.File(verb_lib_path, mode="r+")    
    clouds = np.load(osp.join(data_dir, demo_info["seg_file"]))
    verb_lib[demo_path].create_group("object_clouds")
    for (i,arg_name) in enumerate(verb_info[verb]["arg_names"]):
        verb_lib[demo_path]["object_clouds"][str(i)] = clouds[i].astype('float32')
    verb_lib.close()
