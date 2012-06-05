#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('topic',)
parser.add_argument('--writer', help='writer class')
parser.add_argument('--out', help='output topic name')
parser.add_argument('--verbose','-v',action='store_true',help='verbose mode')
parser.add_argument('--downsample','-d',type=int,default=1,help='downsample by ratio')
args = parser.parse_args()



import comm.comm_ros as cr
from comm import comm
import roslib
roslib.load_manifest('rospy');
import rospy
from time import sleep


if args.out is None: outTopic = args.topic.strip('/')
else: outTopic = args.out

comm.initComm()
rosMsgType = cr.getTopicType(args.topic)
if args.writer is not None: writerClass = writerClassFromString(args.writer)
else: writerClass = cr.writerClassFromType(rosMsgType)


rospy.init_node('write_topic',anonymous=True, disable_signals=True)

writer = writerClass(outTopic, writerClass.extension,args.topic,rosMsgType,verbose=args.verbose, downsample = args.downsample)
writer.start()
    
    
try:
    sleep(10**10)
except KeyboardInterrupt:
    print "exiting"
