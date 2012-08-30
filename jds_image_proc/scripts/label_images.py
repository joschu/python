#!/usr/bin/env python

"""
e.g. python ~/python/jds_image_proc/label_images.py -i jpgs/*.jpg -m once/roi_mask.png -o jpgs2 -l jpgs2/*.png

"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("-i","--images", help = "images to label", nargs = "+")
parser.add_argument("-l","--labels", help = "labels", nargs = "*")
parser.add_argument("-m","--mask", help = "maskfile")
parser.add_argument("-o","--outdir", help = "output directory")
parser.add_argument("-d","--downsample", help = "downsample images before processing", type=float, default=1)
parser.add_argument("--minpix", help = "all smaller regions will be removed in postprocessing", type=int, default=50)
parser.add_argument("--classifier_only",help="just train a classifier",action="store_true")

args = parser.parse_args()
print args.downsample
import jds_image_proc.smart_labeler as sl

assert args.labels is None or len(args.labels) == len(args.images)
assert args.mask is not None
assert args.outdir is not None
print args.mask

ml = sl.MultiLabelerWithClassifier(args.images, args.mask, args.labels, downsample = args.downsample, min_pix = args.minpix)

if args.classifier_only:
    ml.trainCls()
    ml.writeFiles(args.outdir,writeLabels=False)
else:
    while not ml.wantsExit():
        ml.step()
    ml.writeFiles(args.outdir)
