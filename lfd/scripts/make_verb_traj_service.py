#!/usr/bin/env python

"""
Given a verb and a bunch of point clouds, spits out a trajectory
"""


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--test", action="store_true")
args = parser.parse_args()


from brett2.ros_utils import RvizWrapper,Marker,pc2xyzrgb,xyz2pc
import h5py, rospy
import lfd
import os.path as osp
from lfd import warping, registration
import roslib
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import MakeTrajectoryRequest, MakeTrajectoryResponse, MakeTrajectory
import geometry_msgs.msg as gm
from numpy import asarray
import numpy as np
from utils import conversions
import yaml
from image_proc.clouds import voxel_downsample


h5file = h5py.File(osp.join(osp.dirname(lfd.__file__),"data","verbs.h5"),"r")
with open(osp.join(osp.dirname(lfd.__file__),"data","verb_info.yaml"),"r") as fh:
    verb_info = yaml.load(fh)
with open(osp.join(osp.dirname(lfd.__file__),"data","tool_info.yaml"),"r") as fh:
    tool_info = yaml.load(fh)

def xyzs_quats_to_poses(xyzs, quats):
    poses = []
    for (xyz, quat) in zip(xyzs, quats):
        poses.append(gm.Pose(gm.Point(*xyz), gm.Quaternion(*quat)))
    return poses

def callback(req):
    assert isinstance(req, MakeTrajectoryRequest)
    if req.verb not in h5file:
        rospy.logerr("verb %s was not found in library",req.verb)
    
    demo_group = h5file[req.verb]["demos"]
    new_object_clouds = [pc2xyzrgb(cloud)[0] for cloud in req.object_clouds]
    
    rospy.logwarn("using first demo")
    best_demo_name = demo_group.keys()[0]
    best_demo = demo_group[best_demo_name]
                           
    assert "transform" in verb_info[req.verb]                       
        
    transform_type = verb_info[req.verb]["transform"]
    x_nd = voxel_downsample(asarray(best_demo["object_clouds"].values()[0]),.02)
    y_md = voxel_downsample(new_object_clouds[0],.02)
    
    if transform_type == "tps":
        warp = registration.tps_rpm(x_nd, y_md, plotting = 4, reg_init = 2, reg_final = .1, n_iter = 39)
    elif transform_type == "translation2d":
        warp = registration.Translation2d()
        warp.fit(x_nd, y_md)
    elif transform_type == "rigid2d":
        warp = registration.Rigid2d()
        warp.fit(x_nd, y_md)
    else:
        raise Exception("transform type %s is not yet implemented"%transform_type)        

    this_verb_info = verb_info[req.verb]
    l_offset,r_offset = np.zeros(3), np.zeros(3)
    if "r_tool" in this_verb_info:
        r_offset = asarray(tool_info[this_verb_info["r_tool"]]["translation"])
    if "l_tool" in this_verb_info:
        l_offset = asarray(tool_info[this_verb_info["l_tool"]]["translation"])
    if "r_offset" in this_verb_info:
        r_offset += asarray(this_verb_info["r_offset"])
    if "l_offset" in this_verb_info:
        l_offset += asarray(this_verb_info["l_offset"])



    arms_used = best_demo["arms_used"].value
    warped_demo = warping.transform_demo(warp, best_demo,arms_used in 'lb', arms_used in 'rb',object_clouds=True, l_offset = l_offset, r_offset = r_offset)
    
    

    resp = MakeTrajectoryResponse()
    traj = resp.traj
        
    if arms_used == "l":
        traj.arms_used = "l"
        traj.gripper0_poses.poses = xyzs_quats_to_poses(warped_demo["l_gripper_xyzs"], warped_demo["l_gripper_quats"])
        traj.gripper0_angles = warped_demo["l_gripper_angles"]
        if "l_tool" in this_verb_info: traj.gripper0_angles *= 0
    elif arms_used == "r":
        traj.arms_used = "r"
        traj.gripper0_poses.poses = xyzs_quats_to_poses(warped_demo["r_gripper_xyzs"], warped_demo["r_gripper_quats"])
        traj.gripper0_angles = warped_demo["r_gripper_angles"]
        if "r_tool" in this_verb_info: traj.gripper0_angles *= 0
    elif arms_used == "b":
        traj.arms_used = "b"
        traj.gripper0_poses.poses = xyzs_quats_to_poses(warped_demo["l_gripper_xyzs"], warped_demo["l_gripper_quats"])
        traj.gripper1_poses.poses = xyzs_quats_to_poses(warped_demo["r_gripper_xyzs"], warped_demo["r_gripper_quats"])
        traj.gripper0_angles = warped_demo["l_gripper_angles"]
        traj.gripper1_angles = warped_demo["r_gripper_angles"]

        if "l_tool" in this_verb_info: traj.gripper0_angles *= 0
        if "r_tool" in this_verb_info: traj.gripper1_angles *= 0
        
    traj.gripper0_poses.header.frame_id = req.object_clouds[0].header.frame_id
    traj.gripper1_poses.header.frame_id = req.object_clouds[0].header.frame_id

    Globals.handles = []
    plot_original_and_warped_demo(best_demo, warped_demo, traj)
    return resp
    
def plot_original_and_warped_demo(best_demo, warped_demo, traj):
    arms_used = best_demo["arms_used"].value

    if arms_used in "lb":
        pose_array = conversions.array_to_pose_array(asarray(best_demo["l_gripper_xyzs"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1),ns = "make_verb_traj_service"))
        pose_array = conversions.array_to_pose_array(asarray(warped_demo["l_gripper_xyzs"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (0,1,0,1),ns = "make_verb_traj_service"))
        
    if arms_used in "rb":
        pose_array = conversions.array_to_pose_array(asarray(best_demo["r_gripper_xyzs"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1),ns = "make_verb_traj_service"))
        pose_array = conversions.array_to_pose_array(asarray(warped_demo["r_gripper_xyzs"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (0,1,0,1),ns = "make_verb_traj_service"))

    Globals.handles.extend(Globals.rviz.draw_trajectory(traj.gripper0_poses, traj.gripper0_angles, ns = "make_verb_traj_service_grippers"))
    if arms_used == 'b':
        Globals.handles.extend(Globals.rviz.draw_trajectory(traj.gripper1_poses, traj.gripper1_angles, ns = "make_verb_traj_service_grippers"))
        

    for (clouds,rgba) in [(sorted_values(best_demo["object_clouds"]),(1,0,0,.5)),(sorted_values(warped_demo["object_clouds"]),(0,1,0,.5))]:

        cloud = []
        for subcloud in clouds:
            cloud.extend(np.asarray(subcloud).reshape(-1,3))
        cloud = np.array(cloud)
        
        cloud = voxel_downsample(cloud, .02)
        pose_array = conversions.array_to_pose_array(cloud, 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = rgba,width=.01,type=Marker.CUBE_LIST))

        
    

def sorted_values(d):
    return [d[key] for key in sorted(d.keys())]

class Globals:
    handles = []
    rviz = None
    @staticmethod
    def setup():
        print "setup"
        Globals.rviz = RvizWrapper.create()
        
    



if __name__ == "__main__":
    if args.test:        
        if rospy.get_name() == "/unnamed": 
            rospy.init_node("test_get_verb_traj_service",disable_signals=True)
        Globals.setup()            
        req = MakeTrajectoryRequest()
        req.verb = "push"
        #import h5py
        F=h5py.File("/home/joschu/python/lfd/data/verbs.h5","r")
        object_clouds = [asarray(cloud) for cloud in sorted_values(F["cut"]["demos"]["0"]["object_clouds"])]
        for i in xrange(len(object_clouds)):
            cloud = object_clouds[i].reshape(-1,3)
            translation = np.random.randn(1,3) * np.array([[.1,.1,0]])
            print "translation", translation
            cloud += translation
            req.object_clouds.append(xyz2pc(cloud,'base_footprint'))
        callback(req)
    else:
        rospy.init_node("make_verb_traj_service",disable_signals=True)
        Globals.setup()
        service = rospy.Service("make_verb_traj", MakeTrajectory, callback)
        rospy.spin()   