import jds_image_proc.interactive_roi as roi
import os,cv2
from os.path import dirname, abspath,join
from jds_image_proc import curves
import numpy as np
from numpy.linalg import norm

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("outfile")
args = parser.parse_args()

assert os.path.exists(os.path.dirname(args.outfile))


table_corners = np.loadtxt("/home/joschu/bulletsim/data/knots/table_corners.txt");

dx_table = table_corners[1] - table_corners[0]
dy_table = table_corners[3] - table_corners[0]
z = table_corners[0,2]

nrows = 300
length_per_pix = norm(dx_table) / nrows
ncols = norm(dy_table) / length_per_pix


img = np.zeros((nrows, ncols),'uint8')+255

nout = 100

poly = roi.get_polyline(img,"curves")
curve = curves.unif_resample(poly,nout,tol=0)

curve3d = (table_corners[0]+np.r_[0,0,.03])[None,:] + np.c_[curve * length_per_pix, np.zeros((nout,1))]
print curve3d
np.savetxt(args.outfile,curve3d,fmt="%.3f")
