import argparse
parser = argparse.ArgumentParser()
parser.add_argument("xyz",type=float,nargs=3)
args = parser.parse_args()

import rospy
import numpy as np
import roslib; roslib.load_manifest("verb_msgs")
from brett2.ros_utils import xyzrgb2pc
from verb_msgs.srv import *
rospy.init_node("call_push_service")
rospy.wait_for_service("/push")
proxy = rospy.ServiceProxy("/push",Push)
request = PushRequest()
xyz = np.array([args.xyz])
rgb = np.array([[0,0,0]])
pc = xyzrgb2pc(xyz,rgb, 'base_link')
request.point_cloud = pc
proxy.call(request)