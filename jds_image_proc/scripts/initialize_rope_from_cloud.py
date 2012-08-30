#!/usr/bin/env python
import argparse, cv2, os

parser = argparse.ArgumentParser()
parser.add_argument("pcd_file")
parser.add_argument("outfile")
parser.add_argument("--plotting",action="store_true")
args = parser.parse_args()

from jds_image_proc import pcd_io
from jds_image_proc.rope_initialization_lite import initialize_rope_from_cloud,RopeInitMessage


xyz,bgr = pcd_io.load_xyzrgb(args.pcd_file)

# get rope mask
xyzs_unif,labels = initialize_rope_from_cloud(xyz, plotting=args.plotting)
if not os.path.exists(os.path.dirname(args.outfile)): os.mkdir(dirname(args.outfile))
print "writing",args.outfile
RopeInitMessage((xyzs_unif,labels)).toFiles(args.outfile,"/tmp/junk.json")
