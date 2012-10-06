
from brett2.ros_utils import RvizWrapper,Marker,pc2xyzrgb,xyz2pc,get_transform
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
from lfd import verbs, tps
from jds_utils.conversions import quats2mats, mats2quats

#with open(osp.join(osp.dirname(lfd.__file__),"data","tool_info.yaml"),"r") as fh:
    #tool_info = yaml.load(fh)

class Globals:
    handles = []
    rviz = None
    @staticmethod
    def setup():
        Globals.rviz = RvizWrapper.create()
        
    


def xyzs_quats_to_poses(xyzs, quats):
    poses = []
    for (xyz, quat) in zip(xyzs, quats):
        poses.append(gm.Pose(gm.Point(*xyz), gm.Quaternion(*quat)))
    return poses

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
        warp = registration.tps_rpm_zrot(x_nd, y_md, plotting=2,reg_init=2,reg_final=.05, n_iter=10, verbose=False)
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
        traj.l_gripper_poses.poses = xyzs_quats_to_poses(warped_demo_data["l_gripper_tool_frame"]["position"], warped_demo_data["l_gripper_tool_frame"]["orientation"])
        traj.l_gripper_angles = warped_demo_data["l_gripper_joint"]
        traj.l_gripper_poses.header.frame_id = req.object_clouds[0].header.frame_id
        if "l_tool" in best_demo_info: traj.l_gripper_angles *= 0
    if arms_used in "rb":
        traj.r_gripper_poses.poses = xyzs_quats_to_poses(warped_demo_data["r_gripper_tool_frame"]["position"], warped_demo_data["r_gripper_tool_frame"]["orientation"])
        traj.r_gripper_angles = warped_demo_data["r_gripper_joint"]
        traj.r_gripper_poses.header.frame_id = req.object_clouds[0].header.frame_id
        if "r_tool" in best_demo_info: traj.r_gripper_angles *= 0
        

    Globals.handles = []
    plot_original_and_warped_demo(best_demo_data, warped_demo_data, traj)
    

    pose_array = conversions.array_to_pose_array(y_md, 'base_footprint')
    Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (0,0,1,1),width=.01,type=Marker.CUBE_LIST))
    return resp

# get the transformation from old_object_cloud to new_object_cloud linearized at lin_point
def get_transformation(old_object_cloud, new_object_cloud, lin_point):
    lin_ag, trans_g, w_ng = tps.tps_fit(old_object_cloud, new_object_cloud, 0.01, 0.01)
    trans = tps_eval([lin_point], lin_ag, trans_g, w_ng, new_object_cloud)
    orien = tps_grad([lin_point, lin_ag, trans_g, w_ng, new_object_cloud)
    return trans_and_orien_to_rigid(trans, orien)

# trans is a 3-element vector, orien is a 3x3 rotation matrix
def trans_and_orien_to_rigid(trans, orien):
    top = numpy.hstack((orien, numpy.transpose([trans])))
    return numpy.vstack(top, [[0, 0, 0, 1]])

def rigid_to_trans_and_orien(rigid):
    trans = rigid[:3, 3]
    orien = rigid[:3, :3]
    return trans, orien
    
# make trajectory for a certain stage of a task
# current_stage_info is the demo information to use
# stage_num is the current task stage number; previous information is unused if this is zero
# previous_new_cloud is the point cloud of the object from the previous stage in the gripper frame
# 'prev' is for the previous stage; 'demo' and 'new' are for demonstration and new current situations, respectively
def make_traj_multi_stage(req, current_stage_info, stage_num, previous_stage_info, previous_new_clouds):

    assert isinstance(req, MakeTrajectoryRequest)

    verb_stage_data = verbs.get_demo_data(current_stage_info.stage_name)

    if stage_num > 0:
        prev_stage_data = verbs.get_demo_data(previous_stage_info.stage_name)
        prev_demo_pc = prev_stage_data["object_clouds"][previous_stage_info.item]["xyz"]
        prev_new_pc = [pc2xyzrgb(cloud)[0] for cloud in previous_new_clouds][0]
        prev_demo_pc_down = voxel_downsample(prev_demo_pc, .02)
        prev_new_pc_down = voxel_downsample(prev_new_pc, .02)

        # transform point cloud in base frame to gripper frame
        # assume right hand has the tool for now
        # use the last pose of the gripper in the stage to figure out the point cloud of the tool in the gripper frame when the tool was grabbed
        prev_demo_gripper_pos = prev_stage_data["r_gripper_tool_frame"]["position"][-1]
        prev_demo_gripper_orien = prev_stage_data["r_gripper_tool_frame"]["orientation"][-1]
        prev_demo_gripper_to_base_transform = trans_and_orien_to_rigid(prev_demo_gripper_pos, prev_demo_gripper_orien)
        prev_demo_base_to_gripper_transform = np.linalg.inv(prev_demo_gripper_to_base_transform)
        prev_demo_pc_in_gripper_frame = [apply_transform(prev_demo_base_to_gripper_transform, point) for point in prev_demo_pc_down]

        # get the new point cloud in the new gripper frame
        (prev_new_base_to_gripper_trans, prev_new_base_to_gripper_rot) = get_transform("base_footprint", "r_gripper")
        prev_new_base_to_gripper_transform = trans_and_orien_to_rigid(prev_new_base_to_gripper_trans, prev_new_base_to_gripper_rot)
        prev_new_pc_in_gripper_frame = [apply_transform(prev_new_base_to_gripper_transform, point) for point in prev_new_pc_down]

        # get the transformation from the new point cloud to the old point cloud for the previous stage
        prev_new_to_old_grip_transform = get_transformation(prev_new_pc_in_gripper_frame, prev_demo_pc_in_gripper_frame, prev_stage_data.special_point)

    old_object_clouds = [verb_stage_data["object_clouds"][obj_name]["xyz"]
            for obj_name in verb_stage_data["object_clouds"].keys()]
    if len(old_object_clouds) > 1:
        raise Exception("i don't know what to do with multiple object clouds")
    
    new_object_clouds = [pc2xyzrgb(cloud)[0] for cloud in req.object_clouds]
    
    x_nd = voxel_downsample(old_object_clouds[0], .02)
    y_md = voxel_downsample(new_object_clouds[0], .02)

    # transformation from old target object to new target wobject in world frame
    current_item_old_to_new_transform = get_transformation(x_nd, y_md, prev_stage_data.special_point)

    # assume only right arm is used for now
    arms_used = current_stage_info.arms_used

    # turn trajectory points into rigid transformation matrices so can apply warping to them
    prev_target_trajectory_mats = [get_transformation(t, o) for t, o in zip(verb_stage_data["r_gripper_tool_frame"]["position"], quats2mats(verb_stage_data["r_gripper_tool_frame"]["orientation"]))]
    new_target_trajectory_mats = [current_item_old_to_new_transform*traj_mat*prev_new_to_old_grip_transform for traj_mat in prev_target_trajectory_mats]

    warped_stage_data = group_to_dict(verb_stage_data) # deep copy it
    warped_stage_data["r_gripper_tool_frame"]["position"] = []
    warped_stage_data["r_gripper_tool_frame"]["orientation"] = []
    for new_traj_mat in new_target_trajectory_mats:
        new_trans, new_orien = rigid_to_trans_and_orien(new_traj_mat)
        warped_stage_data["r_gripper_tool_frame"]["position"].append(new_trans)
        warped_stage_data["r_gripper_tool_frame"]["orientation"].append(new_orien)

    resp = MakeTrajectoryResponse()
    traj = resp.traj
        
    traj.r_gripper_poses.poses = xyzs_quats_to_poses(warped_demo_data["r_gripper_tool_frame"]["position"], warped_demo_data["r_gripper_tool_frame"]["orientation"])
    traj.r_gripper_angles = warped_demo_data["r_gripper_joint"]
    traj.r_gripper_poses.header.frame_id = req.object_clouds[0].header.frame_id
    if "r_tool" in best_stage_info: traj.r_gripper_angles *= 0
        
    Globals.handles = []
    plot_original_and_warped_demo(verb_stage_data, warped_demo_data, traj)
    
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
