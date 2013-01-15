from brett2.ros_utils import RvizWrapper,Marker,pc2xyzrgb,xyz2pc
import brett2.ros_utils as ru
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
from jds_utils import conversions
import yaml
from jds_image_proc.clouds import voxel_downsample
from lfd import verbs
from lfd import tps
from jds_utils.conversions import quats2mats, mats2quats, mat2quat

import jds_utils.transformations as jut
import jds_utils.conversions as juc
from registration import orthogonalize3_cross
from utils_lfd import group_to_dict
from jds_utils.yes_or_no import yes_or_no

class Globals:
    handles = []
    rviz = None
    @staticmethod
    def setup():
        Globals.rviz = RvizWrapper.create()

def make_traj(req):
    """
    Generate a trajectory using warping
    See MakeTrajectory service description
    (TODO) should be able to specify a specific demo
    """
    assert isinstance(req, MakeTrajectoryRequest)
    
    new_object_clouds = [pc2xyzrgb(cloud)[0] for cloud in req.object_clouds]
    
    scene_info = "PLACEHOLDER"
    best_demo_name, best_demo_info = verbs.get_closest_demo(req.verb, scene_info)    
    best_demo_data = verbs.get_demo_data(best_demo_name)
        
    transform_type = "tps"
    
    old_object_clouds = [best_demo_data["object_clouds"][obj_name]["xyz"]
            for obj_name in best_demo_data["object_clouds"].keys()]

    if len(old_object_clouds) > 1:
        raise Exception("i don't know what to do with multiple object clouds")
    x_nd = voxel_downsample(old_object_clouds[0],.02)
    y_md = voxel_downsample(new_object_clouds[0],.02)
    
    if transform_type == "tps":
        #warp = registration.tps_rpm_zrot(x_nd, y_md, plotting=2,reg_init=2,reg_final=.05, n_iter=10, verbose=False)
        warp = registration.tps_rpm(x_nd, y_md, plotting=2,reg_init=2,reg_final=.05, n_iter=10, verbose=False)
    elif transform_type == "translation2d":
        warp = registration.Translation2d()
        warp.fit(x_nd, y_md)
    elif transform_type == "rigid2d":
        warp = registration.Rigid2d()
        warp.fit(x_nd, y_md)
    else:
        raise Exception("transform type %s is not yet implemented"%transform_type)        

    l_offset,r_offset = np.zeros(3), np.zeros(3)
    #if "r_tool" in best_demo_info:
        #r_offset = asarray(tool_info[this_verb_info["r_tool"]]["translation"])
    #if "l_tool" in best_demo_info:
        #l_offset = asarray(tool_info[this_verb_info["l_tool"]]["translation"])


    arms_used = best_demo_info["arms_used"]
    warped_demo_data = warping.transform_verb_demo(warp, best_demo_data)        

    resp = MakeTrajectoryResponse()
    traj = resp.traj

    traj.arms_used = arms_used
    if arms_used in "lb":
        traj.l_gripper_poses.poses = juc.xyzs_quats_to_poses(warped_demo_data["l_gripper_tool_frame"]["position"], warped_demo_data["l_gripper_tool_frame"]["orientation"])
        traj.l_gripper_angles = warped_demo_data["l_gripper_joint"]
        traj.l_gripper_poses.header.frame_id = req.object_clouds[0].header.frame_id
        if "l_tool" in best_demo_info: traj.l_gripper_angles *= 0
    if arms_used in "rb":
        traj.r_gripper_poses.poses = juc.xyzs_quats_to_poses(warped_demo_data["r_gripper_tool_frame"]["position"], warped_demo_data["r_gripper_tool_frame"]["orientation"])
        traj.r_gripper_angles = warped_demo_data["r_gripper_joint"]
        traj.r_gripper_poses.header.frame_id = req.object_clouds[0].header.frame_id
        if "r_tool" in best_demo_info: traj.r_gripper_angles *= 0

    Globals.handles = []
    plot_original_and_warped_demo(best_demo_data, warped_demo_data, traj)

    pose_array = conversions.array_to_pose_array(y_md, 'base_footprint')
    Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (0,0,1,1),width=.01,type=Marker.CUBE_LIST))
    return resp

def plot_original_and_warped_demo(best_demo, warped_demo, traj):
    arms_used = best_demo["arms_used"]

    if arms_used in "lb":
        pose_array = conversions.array_to_pose_array(asarray(best_demo["l_gripper_tool_frame"]["position"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1),ns = "make_verb_traj_service"))
        pose_array = conversions.array_to_pose_array(asarray(warped_demo["l_gripper_tool_frame"]["position"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (0,1,0,1),ns = "make_verb_traj_service"))
        
    if arms_used in "rb":
        pose_array = conversions.array_to_pose_array(asarray(best_demo["r_gripper_tool_frame"]["position"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,0,0,1),ns = "make_verb_traj_service"))
        pose_array = conversions.array_to_pose_array(asarray(warped_demo["r_gripper_tool_frame"]["position"]), 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (0,1,0,1),ns = "make_verb_traj_service"))

    Globals.handles.extend(Globals.rviz.draw_trajectory(traj.l_gripper_poses, traj.l_gripper_angles, ns = "make_verb_traj_service_grippers"))
    if arms_used == 'b':
        Globals.handles.extend(Globals.rviz.draw_trajectory(traj.r_gripper_poses, traj.r_gripper_angles, ns = "make_verb_traj_service_grippers"))
        

    for (clouds,rgba) in [(sorted_values(best_demo["object_clouds"]),(1,0,0,.5)),
                          (sorted_values(warped_demo["object_clouds"]),(0,1,0,.5))]:

        cloud = []
        for subcloud in clouds:
            cloud.extend(np.asarray(subcloud["xyz"]).reshape(-1,3))
        cloud = np.array(cloud)
        
        cloud = voxel_downsample(cloud, .02)
        pose_array = conversions.array_to_pose_array(cloud, 'base_footprint')
        Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = rgba,width=.01,type=Marker.CUBE_LIST))

def sorted_values(d):
    return [d[key] for key in sorted(d.keys())]
