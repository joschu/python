#!/usr/bin/env python
from image_proc import interactive_roi as roi
import os,cv2
from os.path import dirname, abspath,join
from image_proc import pcd_io

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-i','--infile')
parser.add_argument('-k','--kinect',action="store_true")
parser.add_argument('-p','--pcdfile')
parser.add_argument('-o','--outfile')
args = parser.parse_args()

if args.infile is not None:
    bgr = cv2.imread(args.infile)
elif args.kinect != False:
    import freenect
    bgr = freenect.sync_get_video()[0][:,:,::-1]
elif args.pcdfile != False:
    _,bgr = pcd_io.load_xyzrgb(args.pcdfile)

else: raise Exception("either specify input image or image source")
assert args.outfile is not None


poly = roi.get_polygon_and_prompt(bgr,'roi')
mask = roi.mask_from_poly(poly,bgr.shape[:2])

print "writing",args.outfile
successCode = cv2.imwrite(args.outfile,mask)
if not successCode: raise Exception("failed to write to %s"%args.outfile)
