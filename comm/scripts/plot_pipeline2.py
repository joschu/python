#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("pipeline")
parser.add_argument("--target")

args = parser.parse_args()

from comm import comm, pipeline
from comm.vision_pipeline import make_towel_pipeline, make_robot_rope_pipeline
import os

if args.pipeline == "robot_rope":
    P = make_robot_rope_pipeline("")

comm.initComm()
os.chdir(comm.DATA_ROOT)

plot = pipeline.PipelinePlot(P)
plot.loop()



