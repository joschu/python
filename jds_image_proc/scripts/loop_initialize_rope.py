#!/usr/bin/env python
import argparse, cv2, os

parser = argparse.ArgumentParser()
parser.add_argument("pcd")
parser.add_argument("label")
parser.add_argument("out")
parser.add_argument("pause",type=float)
parser.add_argument("--plotting",action="store_true")
args = parser.parse_args()


from jds_image_proc import pcd_io
from jds_image_proc.rope_initialization import initialize_rope, RopeInitMessage
from comm import comm
from time import time,sleep
from glob import glob
from os.path import basename
from jds_utils.colorize import colorize
import traceback

comm.initComm()

def getLastInd(topic):
    infoFile = max(glob(comm.filePath("info*.json",topic)))
    return int(basename(infoFile)[4:16])
    

pcdGetter = comm.FileGetter(args.pcd,"pcd",comm.PointCloudMessage)
labelGetter = comm.FileGetter(args.label,"bmp",comm.SingleChannelImageMessage)
ropeInitPub = comm.FilePublisher(args.out,"txt")

prev_id = -1
while True:

    
    latest_id = getLastInd(args.label)
    print "latest id", latest_id
    if latest_id == prev_id:        
        sleep(.1)
        continue
    else:
        prev_id = latest_id    
    
    timePrevStart = time()

    xyz,bgr = pcdGetter.recv_id(latest_id).data
    label = labelGetter.recv_id(latest_id).data

    if label is None: raise Exception("could not read label file")
        
    try:
        xyzs,labels = initialize_rope(label,xyz, bgr,plotting=args.plotting)
        ropeInitPub.send(RopeInitMessage(data=(xyzs, labels)))    
        sleep(max(args.pause - (time() - timePrevStart),0))
    except Exception:
        print colorize("exception occurred in rope init:","red")
        traceback.print_exc()
    
