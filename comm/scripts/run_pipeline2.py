#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--order",choices=["series","parallel"])
parser.add_argument("--reset",choices=["no","full","partial"])
parser.add_argument("--noclean",action="store_true")
parser.add_argument("pipeline",choices = ["robot_rope", "human_towel","human_rope"])
parser.add_argument("--target")


args = parser.parse_args()
assert args.order is not None
assert args.reset is not None

from comm.vision_pipeline import make_towel_pipeline, make_robot_rope_pipeline2, make_human_rope_pipeline
from comm import pipeline
from comm import comm
import os, subprocess
from jds_utils.yes_or_no import yes_or_no

comm.initComm()
os.chdir(comm.DATA_ROOT)



if args.pipeline == "robot_rope":

    if args.reset == "full":
        yn = yes_or_no("are you sure you want to delete everything?")
        if yn: subprocess.check_call("rm -r *", shell=True)
        else: exit(0)
    elif args.reset == "partial":
        subprocess.check_call("rm -rf kinect labels images rope_pts towel_pts rope_init rope_model logs/* joint_states base_pose once/table_corners.txt",shell=True)
    elif args.reset == "no":
        pass
    PIPELINE = make_robot_rope_pipeline2(
        classifier="/home/joschu/python/jds_image_proc/pixel_classifiers/rope_sdh_light_afternoon/classifier.pkl", 
        downsample=5)
    

if args.pipeline == "human_rope":

    if args.reset == "full":
        yn = yes_or_no("are you sure you want to delete everything?")
        if yn: subprocess.check_call("rm -r *", shell=True)
        else: exit(0)
    elif args.reset == "partial":
        subprocess.check_call("rm -rf kinect labels rope_pts  rope_init rope_model logs/* once/table_corners.txt",shell=True)
    elif args.reset == "no":
        pass
    PIPELINE = make_human_rope_pipeline(downsample=5)

    
elif args.pipeline == "human_towel":
    PIPELINE = make_towel_pipeline(downsample = 10)


    if args.reset == "full":
        subprocess.check_call("rm -r *", shell=True)
    elif args.reset == "partial":
        subprocess.check_call("rm -rf kinect labels rope_pts towel_pts rope_init rope_model towel_model once/table_corners.txt once/init_rope.txt logs/*",shell=True)
    elif args.reset == "no":
        pass



if args.target is not None: assert args.target in PIPELINE.get_items()
comm.initComm()

TARGET_PIPELINE = PIPELINE.restrict_to_target(args.target) if args.target is not None else PIPELINE

if args.order == "series":
    pipeline.execute_series(TARGET_PIPELINE)        
elif args.order == "parallel":
    pipeline.execute_parallel(TARGET_PIPELINE, lifetime = 10, max_lag = 5, noclean = args.noclean)
