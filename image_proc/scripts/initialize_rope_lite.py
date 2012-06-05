#!/usr/bin/env python
import argparse, cv2, os

parser = argparse.ArgumentParser()
parser.add_argument("pcd_file")
parser.add_argument("label_file")
parser.add_argument("outfile")
parser.add_argument("--plotting",action="store_true")
args = parser.parse_args()

from image_proc import pcd_io
from image_proc.rope_initialization_lite import initialize_rope,RopeInitMessage


xyz,bgr = pcd_io.load_xyzrgb(args.pcd_file)

# get rope mask
label_img = cv2.imread(args.label_file,0)
if label_img is None: raise Exception("could not read image %s"%args.label_file)

xyzs_unif,labels = initialize_rope(label_img,xyz, bgr,plotting=args.plotting)
if not os.path.exists(os.path.dirname(args.outfile)): os.mkdir(dirname(args.outfile))
print "writing",args.outfile
RopeInitMessage((xyzs_unif,labels)).toFiles(args.outfile,"/tmp/junk.json")
