import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--fmt",choices=["txt","npy"],default="txt")
parser.add_argument("cloud_in")
args = parser.parse_args()

import roslib;
roslib.load_manifest("rospy")
import rospy
rospy.init_node("write_clouds",disable_signals = True)

from brett2.ros_utils import pc2xyzrgb
import sensor_msgs.msg as sm
import numpy as np

while not rospy.is_shutdown():
    fname = raw_input("filename?: ")
    
    print "waiting for point cloud on input topic"
    msg = rospy.wait_for_message(args.cloud_in, sm.PointCloud2)
    print "ok"
    
    xyz,_ = pc2xyzrgb(msg)
    if args.fmt == "txt": np.savetxt(fname, np.squeeze(xyz))
    elif args.fmt == "npy": np.save(fname, np.squeeze(xyz))
    

