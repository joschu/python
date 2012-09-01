#!/usr/bin/env python
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--live", action="store_true")
parser.add_argument("demo_name",type=str)
args = parser.parse_args()

from lfd import verbs, make_verb_traj, exec_verb_traj
from verb_msgs.srv import *
import rospy
import numpy as np
import brett2.ros_utils as ru
import roslib; roslib.load_manifest("snazzy_msgs")
from snazzy_msgs.srv import *
import sensor_msgs.msg as sm
from jds_utils.yes_or_no import yes_or_no

if rospy.get_name() == "/unnamed": 
    rospy.init_node("test_get_verb_traj_service",disable_signals=True)
make_verb_traj.Globals.setup()
exec_verb_traj.Globals.setup()

demo_info = verbs.get_demo_info(args.demo_name)
demo_data = verbs.get_demo_data(args.demo_name)

make_req = MakeTrajectoryRequest()
make_req.verb = demo_info["verb"]


if args.live:
    seg_svc = rospy.ServiceProxy("/interactive_segmentation", ProcessCloud)
    pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)
    pc_tf = ru.transformPointCloud2(pc, exec_verb_traj.Globals.pr2.tf_listener, "base_footprint", pc.header.frame_id)
        
    for obj_name in demo_info["args"]:
        print "select the %s"%obj_name
        pc_sel = seg_svc.call(ProcessCloudRequest(cloud_in = pc_tf)).cloud_out
        make_req.object_clouds.append(pc_sel)
    
else:
    object_clouds = [demo_data["object_clouds"][obj_name]["xyz"]
                     for obj_name in demo_data["object_clouds"].keys()]
    for i in xrange(len(object_clouds)):
        cloud = object_clouds[i].reshape(-1,3)
        translation = np.random.randn(1,3) * np.array([[.1,.1,0]])
        cloud += translation
        make_req.object_clouds.append(ru.xyz2pc(cloud,'base_footprint'))
    
make_resp = make_verb_traj.make_traj(make_req)
yn = yes_or_no("continue?")
if yn:
    exec_req = ExecTrajectoryRequest()
    exec_req.traj = make_resp.traj
    exec_verb_traj.exec_traj(exec_req)
