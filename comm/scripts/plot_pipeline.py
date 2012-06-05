#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--target")

args = parser.parse_args()
assert args.target is not None

from comm.vision_pipeline import make_vision_pipeline
from comm import pipeline
from comm import comm
import os

comm.initComm()
os.chdir(comm.DATA_ROOT)
PIPELINE = make_vision_pipeline(0,0)
assert args.target in PIPELINE.get_items()
plot = pipeline.PipelinePlot(PIPELINE.restrict_to_target(args.target))
plot.loop()



