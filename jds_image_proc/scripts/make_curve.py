#!/usr/bin/env python
from jds_image_proc import interactive_roi as roi
import os,cv2
from os.path import dirname, abspath,join
from jds_image_proc import curves, pcd_io
import numpy as np

import argparse
parser = argparse.ArgumentParser()
parser.add_argument('pcdfile')
parser.add_argument('outfile')
args = parser.parse_args()

xyz,bgr = pcd_io.load_xyzrgb(args.pcdfile)
print xyz.shape, bgr.shape

cv2.namedWindow('draw curve',cv2.cv.CV_WINDOW_NORMAL)
xys = roi.get_polyline(bgr,'draw curve')
uvs = np.int32(xys)[:,::-1]
us,vs = uvs.T
xyzs = xyz[us,vs]
xyzs_good = xyzs[np.isfinite(xyzs).all(axis=1)]
print "%i out of %i labeled points are good"%(len(xyzs_good), len(xyzs))
xyzs_unif = curves.unif_resample(xyzs_good,100,tol=.002)


#if bool(args.outfile) and not os.path.exists(dirname(args.outfile)): os.mkdir(dirname(args.outfile))
print "writing",args.outfile
np.savetxt(args.outfile,xyzs_unif)

