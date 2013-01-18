#!/usr/bin/env python
"""
For annotating a point cloud that corresponds to demonstration. Select which point clouds are the verb "arguments"
"""


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("infile")
parser.add_argument("--outfile")
parser.add_argument("--plotting", action="store_true")
parser.add_argument("--objs",nargs="*",type=str)
parser.add_argument("--do_filtering", action="store_true")
args = parser.parse_args()

from point_clouds import tabletop
import numpy as np
import rospy
import roslib
roslib.load_manifest('tf')
roslib.load_manifest('snazzy_msgs')
import tf
from brett2.ros_utils import pc2xyzrgb
import sensor_msgs.msg as sm
from brett2 import ros_utils
import os
import h5py
from snazzy_msgs.srv import *
from jds_utils import conversions
import rope_vision.rope_initialization as ri

rospy.init_node("manually_segment_point_cloud")
listener = tf.TransformListener()
seg_svc = rospy.ServiceProxy("interactive_segmentation", ProcessCloud)

if args.infile.endswith("npz"):
    f = np.load(args.infile)
    xyz0, rgb0 = f["xyz"], f["bgr"]
    pc = ros_utils.xyzrgb2pc(xyz0, rgb0, "base_footprint")
else:
    raise NotImplementedError

if args.objs is None:
    object_names = raw_input("type object names separated by spaces\n").split()
else:
    object_names = args.objs

outfilename  = os.path.splitext(args.infile)[0] + ".seg.h5"
if os.path.exists(outfilename): os.remove(outfilename)
outfile = h5py.File(outfilename, "w")

def array_contains(arr, pt):
    return np.any(np.all((pt[None,:] == arr), axis=1))

def get_good_inds(orig_xyz, good_xyz):
    good_inds = []
    for i, xyz in enumerate(orig_xyz):
        if array_contains(good_xyzs, xyz):
            good_inds.append(i)
    return good_inds

for object_name in object_names:
    pc_sel = seg_svc.call(ProcessCloudRequest(cloud_in = pc)).cloud_out
    xyz, rgb = ros_utils.pc2xyzrgb(pc_sel)
    xyz = xyz.reshape(-1,3)
    rgb = rgb.reshape(-1,3)
    if args.do_filtering:
        graph = ri.points_to_graph(xyz, .03)
        cc = ri.largest_connected_component(graph)
        good_xyzs = np.array([graph.node[node_id]["xyz"] for node_id in cc.nodes()])
        good_inds = get_good_inds(xyz, good_xyzs)
        good_rgbs = rgb[good_inds]
    else:
        good_xyzs, good_rgbs = xyz, rgb

    outfile.create_group(object_name)
    outfile[object_name]["xyz"] = good_xyzs
    outfile[object_name]["rgb"] = good_rgbs

if args.plotting:
    rviz = ros_utils.RvizWrapper.create()
    rospy.sleep(.5)
    from brett2.ros_utils import Marker
    pose_array = conversions.array_to_pose_array(good_xyzs.reshape(-1,3), "base_footprint")
    plot_handle = rviz.draw_curve(pose_array, rgba = (1,1,0,1), type=Marker.CUBE_LIST, width=.001, ns="segmentation")
    raw_input("press enter when done looking")
