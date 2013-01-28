import argparse
parser = argparse.ArgumentParser()
parser.add_argument("base")
parser.add_argument("other")
args = parser.parse_args()

import numpy as np
import transform_gui
src = np.loadtxt(args.base)
targ = np.loadtxt(args.other)
transformer = transform_gui.CloudAffineTransformer(src,targ)
transformer.configure_traits()
