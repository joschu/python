#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--order",choices=["series","parallel"])
parser.add_argument("--reset",choices=["no","full","partial","allbutkinect"])
parser.add_argument("--target")
parser.add_argument("--downsample",type=int,default=3)
parser.add_argument("--classifier")
parser.add_argument("--init_period", type=float, default=0)
parser.add_argument("--no_cleanup", action="store_true")


args = parser.parse_args()
assert args.order is not None
assert args.reset is not None
assert args.target is not None
assert args.classifier is not None

from comm.vision_pipeline import make_vision_pipeline
from comm import pipeline
from comm import comm
import os

comm.initComm()
os.chdir(comm.DATA_ROOT)
PIPELINE = make_vision_pipeline(downsample=args.downsample, classifier=args.classifier, init_period = args.init_period)
assert args.target in PIPELINE.get_items()

LIFETIME = 9999 if args.no_cleanup else 10


import subprocess
if args.reset == "full":
    comm.resetDataDir()
elif args.reset == "partial":
    subprocess.check_call("rm -rf kinect labels images rope_pts towel_pts rope_init rope_model towel_model once/init_rope.txt",shell=True)
elif args.reset == "allbutkinect":
    subprocess.check_call("rm -rf labels images rope_pts towel_pts rope_init rope_model towel_model once/init_rope.txt",shell=True)    
if args.order == "series":
    pipeline.execute_series(PIPELINE.restrict_to_target(args.target))
elif args.order == "parallel":
    pipeline.execute_parallel(PIPELINE.restrict_to_target(args.target), lifetime = LIFETIME, max_lag = 5)
