#!/usr/bin/env python
import argparse, shutil, os

parser = argparse.ArgumentParser()
parser.add_argument("-d","--dir",type=str,default=os.environ['DATA_ROOT'])
args = parser.parse_args()
assert(args.dir)
if os.path.exists(args.dir):
    answer = raw_input("are you sure you want to remove the whole directory %s? (type y to accept)\n"%args.dir)
    if answer.startswith('y'):
        shutil.rmtree(args.dir)
    else:
        exit(-1)
os.mkdir(args.dir)
os.mkdir(os.path.join(args.dir,"once"))
