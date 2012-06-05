#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("files",nargs="+")
args = parser.parse_args()

for fname in args.files:

    with open(fname,"r") as fh: s = fh.read()
    
    s0,s1 = s[:1000], s[1000:]
    s0 = s0.replace("rgb","rgba")
    s0 = s0.replace("rgbaa","rgba")
    s0 = s0.replace("F F F F","F F F U")
    
    with open(fname,"w") as fh: fh.write(s0+s1)