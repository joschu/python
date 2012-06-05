#!/usr/bin/env python
import argparse, cv2, os

parser = argparse.ArgumentParser()
parser.add_argument("pcd")
parser.add_argument("out")
parser.add_argument("pause",type=float)
parser.add_argument("--plotting",action="store_true")
args = parser.parse_args()


from image_proc import pcd_io
from image_proc.rope_initialization_lite import initialize_rope_from_cloud, RopeInitMessage
from comm import comm
from time import time,sleep
from glob import glob
from os.path import basename
from utils.colorize import colorize
import traceback

comm.initComm()

def getLastInd(topic):
    infoFile = max(glob(comm.filePath("info*.json",topic)))
    return int(basename(infoFile)[4:16])
    

pcdGetter = comm.FileGetter(args.pcd,"pcd",comm.PointCloudMessage)
ropeInitPub = comm.FilePublisher(args.out,"txt")

prev_id = -1
while True:

    
    latest_id = getLastInd(args.pcd)
    print "latest id", latest_id
    if latest_id == prev_id:        
        sleep(.1)
        continue
    else:
        prev_id = latest_id    
    
    timePrevStart = time()

    xyz,bgr = pcdGetter.recv_id(latest_id).data

        
    try:
        xyzs,labels = initialize_rope_from_cloud(xyz, plotting=args.plotting)
        ropeInitPub.send(RopeInitMessage(data=(xyzs, labels)))    
        print "rope initialization successful"
        sleep(max(args.pause - (time() - timePrevStart),0))
    except Exception:
        print colorize("exception occurred in rope init:","red")
        traceback.print_exc()
    
