#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("from_frame")
parser.add_argument("to_frame")
parser.add_argument("comm_topic")
parser.add_argument("--hz", default=20., type=float)
args = parser.parse_args()


from comm import comm
import roslib
roslib.load_manifest('tf');
import rospy, tf
from time import sleep
import numpy as np
import traceback

comm.initComm()
rospy.init_node('tf_to_comm',anonymous=True, disable_signals=True)

listener = tf.TransformListener()
pub = comm.FilePublisher(args.comm_topic,"txt")

while True:
    try:
        (trans,rot) = listener.lookupTransform(args.from_frame, args.to_frame, rospy.Time(0))            
        msg = comm.ArrayMessage(data = np.r_[rot, trans])
        pub.send(msg)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #print "tf error"
        #traceback.print_exc()
        pass

    sleep(1./args.hz)
