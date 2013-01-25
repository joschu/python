import rospy
import roslib
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import MakeTrajectoryRequest, MakeTrajectoryResponse
import brett2.ros_utils as ru
from lfd import registration
import numpy as np
from jds_image_proc.clouds import voxel_downsample
import jds_utils.transformations as jut
import jds_utils.conversions as juc
from utils_lfd import group_to_dict

class Globals:
    handles = []
    rviz = None
    @staticmethod
    def setup():
        Globals.rviz = ru.RvizWrapper.create()

# get the warping transformation specified by transform_type for from_cloud to to_cloud
def get_warping_transform(from_cloud, to_cloud, transform_type="tps"):
    if transform_type == "tps":
        return registration.tps_rpm(from_cloud, to_cloud, plotting=2, reg_init=2, reg_final=.5, n_iter=10, verbose=False)
    elif transform_type == "tps_zrot":
        return registration.tps_rpm_zrot(from_cloud, to_cloud, plotting=2, reg_init=2, reg_final=.5, n_iter=10, verbose=False)
    elif transform_type == "translation2d":
        warp = registration.Translation2d()
        warp.fit(from_cloud, to_cloud)
        return warp
    elif transform_type == "rigid2d":
        warp = registration.Rigid2d()
        warp.fit(from_cloud, to_cloud)
        return warp
    else:
        raise Exception("transform type %s is not yet implemented" % transform_type)

# applies a transformation specified by a 4x4 matrix to a 3d point
# transform is a 4x4 matrix (np.array) and point is a 3-element vector (np.array)
def apply_mat_transform_to_xyz(transform, point):
    applied = np.dot(transform, np.array([point[0], point[1], point[2], 1]))
    return applied[:3]

# applies tps transformation to a frame specified by hmat
def apply_tps_transform_to_hmat(tps_transform, hmat):
    xyz, quat = juc.hmat_to_trans_rot(hmat)
    warped_xyz, warped_mat = tps_transform.transform_frames(np.array([xyz]), np.array([juc.quat2mat(quat)]))
    return juc.trans_rot_to_hmat(warped_xyz[0], juc.mat2quat(warped_mat[0]))

# make a function that transforms a point from the frame of the gripper with the tool to the world frame using tf
def make_world_to_grip_transform_tf(gripper_frame_name):
    def grip_to_world_transform_tf(point):
        return ru.transform_points(np.array([point]), ru.get_tf_listener(), gripper_frame_name, "base_footprint")[0]
    return grip_to_world_transform_tf

# make a function that transforms a point from the frame of the gripper with the tool to the world frame the supplied hmat
def make_world_to_grip_transform_hmat(world_to_grip_hmat):
    def grip_to_world_transform_hmat(point):
        return apply_mat_transform_to_xyz(world_to_grip_hmat, point)
    return grip_to_world_transform_hmat

def print_hmat_info(hmat, title):
    print title
    print hmat

# gets the warping transformation for a tool in a gripper (arm specifies which gripper, and it should be {l,r})
def get_demo_to_exp_tool_transform(verb_data_accessor, tool_stage_info, exp_tool_cloud, transform_type):
    tool_stage_data = verb_data_accessor.get_demo_stage_data(tool_stage_info.stage_name)
    demo_tool_cloud = tool_stage_data["object_cloud"][tool_stage_info.item]["xyz"]
    demo_tool_cloud_down = voxel_downsample(demo_tool_cloud, .02)
    exp_tool_cloud_down = voxel_downsample(exp_tool_cloud, .02)
    demo_to_exp_tool_transform = get_warping_transform(demo_tool_cloud_down,
                                                       exp_tool_cloud_down,
                                                       transform_type)
    return demo_to_exp_tool_transform

# returns the warping transform for the target object of the current stage
def get_demo_to_exp_target_transform(demo_target_cloud, exp_target_cloud, transform_type):
    demo_target_cloud_down = voxel_downsample(demo_target_cloud, .02)
    exp_target_cloud_down = voxel_downsample(exp_target_cloud, .02)
    demo_to_exp_target_transform = get_warping_transform(demo_target_cloud_down, exp_target_cloud_down, transform_type)
    return demo_to_exp_target_transform

# returns the demo gripper trajectory for the current stage as an array of matrices
def get_demo_grip_traj_mats(target_stage_data, arm):
    gripper_data_key = "%s_gripper_tool_frame" % (arm)
    demo_target_grip_traj_xyzs = target_stage_data[gripper_data_key]["position"]
    demo_target_grip_traj_oriens = target_stage_data[gripper_data_key]["orientation"]
    demo_target_grip_traj_mats = [juc.trans_rot_to_hmat(trans, orien) for (trans, orien) in zip(demo_target_grip_traj_xyzs, demo_target_grip_traj_oriens)]
    return demo_target_grip_traj_mats

# applies the special point translation to a gripper trajectory
def get_demo_spec_pt_traj_mats(demo_target_grip_traj_mats, spec_pt_in_grip):
    if np.all(spec_pt_in_grip == np.zeros(3)):
        return demo_target_grip_traj_mats
    grip_to_spec_pt_trans = jut.translation_matrix(spec_pt_in_grip)
    demo_spec_pt_traj_mats = [np.dot(gripper_mat, grip_to_spec_pt_trans) for gripper_mat in demo_target_grip_traj_mats]
    return demo_spec_pt_traj_mats

# apply the warping transformation to the special point trajectory
def get_warped_spec_pt_traj_mats(demo_spec_pt_traj_mats, demo_to_exp_target_transform):
    demo_spec_pt_traj_xyzs, demo_spec_pt_traj_oriens = [], []
    for demo_spec_pt_traj_mat in demo_spec_pt_traj_mats:
        demo_spec_pt_traj_xyz, demo_spec_pt_traj_orien = juc.hmat_to_trans_rot(demo_spec_pt_traj_mat)
        demo_spec_pt_traj_xyzs.append(demo_spec_pt_traj_xyz)
        demo_spec_pt_traj_oriens.append(juc.quat2mat(demo_spec_pt_traj_orien))
    warped_spec_pt_traj_xyzs, warped_spec_pt_traj_oriens = demo_to_exp_target_transform.transform_frames(np.array(demo_spec_pt_traj_xyzs), np.array(demo_spec_pt_traj_oriens))
    warped_spec_pt_traj_mats = [juc.trans_rot_to_hmat(warped_spec_pt_traj_xyz, juc.mat2quat(warped_spec_pt_traj_orien)) for warped_spec_pt_traj_xyz, warped_spec_pt_traj_orien in zip(warped_spec_pt_traj_xyzs, warped_spec_pt_traj_oriens)]
    return warped_spec_pt_traj_mats

def get_demo_tool_grip_to_world_transform(tool_stage_data, arm):
    gripper_data_key = "%s_gripper_tool_frame" % (arm)
    demo_tool_grip_pos = tool_stage_data[gripper_data_key]["position"][-1]
    demo_tool_grip_orien = tool_stage_data[gripper_data_key]["orientation"][-1]
    demo_tool_grip_to_world_transform = juc.trans_rot_to_hmat(demo_tool_grip_pos, demo_tool_grip_orien)
    return demo_tool_grip_to_world_transform

# gets the new gripper trajectory from the warped special point trajectory
def get_warped_grip_traj_mats(warped_spec_pt_traj_mats, tool_stage_data, demo_to_exp_tool_transform, spec_pt_in_grip, world_to_grip_transform_func, arm):
    if demo_to_exp_tool_transform is None:
        grip_to_spec_pt_transform = np.linalg.inv(jut.translation_matrix(spec_pt_in_grip))
    else:
        demo_tool_grip_to_world_transform = get_demo_tool_grip_to_world_transform(tool_stage_data, arm)
        demo_spec_pt_in_world = apply_mat_transform_to_xyz(demo_tool_grip_to_world_transform, spec_pt_in_grip)
        demo_spec_pt_in_world_frame = jut.translation_matrix(demo_spec_pt_in_world)
        # find the transformation from the new special point to the gripper frame
        warped_spec_pt_in_world_frame = apply_tps_transform_to_hmat(demo_to_exp_tool_transform, demo_spec_pt_in_world_frame)
        warped_spec_pt_in_world_trans, warped_spec_pt_in_world_rot = juc.hmat_to_trans_rot(warped_spec_pt_in_world_frame)
        warped_spec_pt_in_grip_trans = world_to_grip_transform_func(warped_spec_pt_in_world_trans)
        warped_spec_pt_in_grip_rot = warped_spec_pt_in_world_rot
        warped_spec_pt_in_grip_frame = juc.trans_rot_to_hmat(warped_spec_pt_in_grip_trans, warped_spec_pt_in_grip_rot)
        print_hmat_info(warped_spec_pt_in_grip_frame, "warped_spec_pt_in_grip_frame")
        # transform the warped special point trajectory back to a gripper trajectory in the experiment
        grip_to_spec_pt_transform = np.linalg.inv(warped_spec_pt_in_grip_frame)
    warped_grip_traj_mats = [np.dot(spec_pt_mat, grip_to_spec_pt_transform) for spec_pt_mat in warped_spec_pt_traj_mats]
    return warped_grip_traj_mats

# sets fields of traj in the MakeTrajectoryResponse
def set_traj_fields_for_response(warped_stage_data, resp, arm, frame_id):
    traj = resp.traj
    gripper_data_key = "%s_gripper_tool_frame" % (arm)
    gripper_joint_key = "%s_gripper_joint" % (arm)
    if arm == 'r':
        traj.r_gripper_poses.poses = juc.xyzs_quats_to_poses(warped_stage_data[gripper_data_key]["position"], warped_stage_data[gripper_data_key]["orientation"])
        traj.r_gripper_angles = warped_stage_data[gripper_joint_key]
        traj.r_gripper_poses.header.frame_id = frame_id
    elif arm == 'l':
        traj.l_gripper_poses.poses = juc.xyzs_quats_to_poses(warped_stage_data[gripper_data_key]["position"], warped_stage_data[gripper_data_key]["orientation"])
        traj.l_gripper_angles = warped_stage_data[gripper_joint_key]
        traj.l_gripper_poses.header.frame_id = frame_id

# wrapper around make_traj_multi_stage_do_work
def make_traj_multi_stage(req, demo_name, stage_num, tool_stage_info, prev_exp_cloud_pc2, verb_data_accessor, transform_type):
    assert len(req.object_clouds) == 1
    exp_tool_cloud = None if stage_num == 0 else ru.pc2xyzrgb(prev_exp_cloud_pc2)[0]
    exp_target_cloud = ru.pc2xyzrgb(req.object_clouds[0])[0]
    clouds_frame_id = req.object_clouds[0].header.frame_id
    world_to_grip_transform_func = make_world_to_grip_transform_tf("%s_gripper_tool_frame" % current_stage_info.arms_used)
    return make_traj_multi_stage_do_work(demo_name, exp_target_cloud, clouds_frame_id,
                                         stage_num, tool_stage_info, exp_tool_cloud,
                                         verb_data_accessor, world_to_grip_transform_func, transform_type)

# make trajectory for a certain stage of a task
# exp_target_cloud: the point cloud of the target object of the experiment scene in the world frame
# frame_id: frame id to use for the poses returned in the trajectory
# stage_num: current stage number; prev information and world_to_grip_transform_func is unused if this is zero
# exp_tool_cloud: the point cloud of the tool object of the experiment scene in the world frame
# world_to_grip_transform_func: transforms a point in the frame of the gripper holding the tool to one in the world frame must correspond to the gripper holding the tool; if no gripper is using a tool, this can be None
# clouds here should have already been processed into lists of xyzs
# if a tool is used, only one arm must be used for the stage
def make_traj_multi_stage_do_work(demo_name, exp_target_cloud, frame_id, stage_num, tool_stage_info, exp_tool_cloud, verb_data_accessor, world_to_grip_transform_func, transform_type):
    
    current_stage_info = verb_data_accessor.get_stage_info(demo_name, stage_num)
    arms_used = current_stage_info.arms_used

    # make sure this is the first stage (no tool) or only one arm is being used with a tool
    assert stage_num == 0 or (arms_used in ['r', 'l'])

    if stage_num == 0:
        tool_stage_data = None
        # don't do any extra transformation for the first stage
        demo_to_exp_tool_transform = None
        # no special point translation for first stage since no tool yet
        spec_pt_in_grip = np.zeros(3)
    else:
        tool_stage_data = verb_data_accessor.get_demo_stage_data(tool_stage_info.stage_name)
        # make sure that the tool stage only uses one arm (the one with the tool)
        demo_to_exp_tool_transform = get_demo_to_exp_tool_transform(verb_data_accessor, tool_stage_info,
                                                                    exp_tool_cloud, transform_type)
        spec_pt_in_grip = np.zeros(3) if tool_stage_info.special_point is None else tool_stage_info.special_point

    current_stage_data = verb_data_accessor.get_demo_stage_data(current_stage_info.stage_name)

    resp = MakeTrajectoryResponse()
    resp.traj.arms_used = arms_used

    # find the target transformation for the experiment scene
    demo_to_exp_target_transform = get_demo_to_exp_target_transform(current_stage_data["object_cloud"][current_stage_info.item]["xyz"],
                                                                    exp_target_cloud,
                                                                    transform_type)

    warped_stage_data = group_to_dict(current_stage_data) # deep copy it
    arms_used_list = ['r', 'l'] if arms_used == 'b' else [arms_used]
    for arm in arms_used_list:
        gripper_data_key = "%s_gripper_tool_frame" % (arm)
        
        # get the demo gripper trajectory
        demo_target_grip_traj_mats = get_demo_grip_traj_mats(current_stage_data, arm)

        # get the demo special point trajectory by applying the special point translation
        demo_spec_pt_traj_mats = get_demo_spec_pt_traj_mats(demo_target_grip_traj_mats, spec_pt_in_grip)

        # get the warped special point trajectory by applying the target warping transformation to the demo special point trajectory
        warped_spec_pt_traj_mats = get_warped_spec_pt_traj_mats(demo_spec_pt_traj_mats, demo_to_exp_target_transform)

        # get the warped trajectory for the gripper using the tool warping transformation
        warped_grip_traj_mats = get_warped_grip_traj_mats(warped_spec_pt_traj_mats, tool_stage_data,
                                                          demo_to_exp_tool_transform, spec_pt_in_grip,
                                                          world_to_grip_transform_func, arm)

        warped_transs, warped_rots = juc.hmats_to_transs_rots(warped_grip_traj_mats)
        warped_stage_data[gripper_data_key]["position"] = warped_transs
        warped_stage_data[gripper_data_key]["orientation"] = warped_rots

        set_traj_fields_for_response(warped_stage_data, resp, arm, frame_id)

        # save the demo special point traj for plotting
        demo_spec_pt_xyzs, exp_spec_pt_xyzs = [], []
        if stage_num > 0:
            demo_spec_pt_xyzs = juc.hmats_to_transs_rots(demo_spec_pt_traj_mats)[0]
            exp_spec_pt_xyzs = juc.hmats_to_transs_rots(warped_spec_pt_traj_mats)[0]

    del Globals.handles[:]

    # plot the demo and warped special points
    current_spec_pt = current_stage_info.special_point
    # currently, don't know which arm grabbed the tool if both arms were used in a stage
    if stage_num == 0 and current_spec_pt is not None and arms_used in ['l', 'r']:
        plot_demo_and_warped_tool_spec_pt(current_spec_pt, current_stage_data, demo_to_exp_target_transform, arms_used)

    # plot the gripper and special point trajectories (red is demo, green is warped)
    plot_original_and_warped_demo_and_spec_pt(current_stage_data, warped_stage_data,
                                              demo_spec_pt_xyzs, exp_spec_pt_xyzs,
                                              arms_used)

    return resp

# plots the special point of a tool for the tool grab stage
def plot_demo_and_warped_tool_spec_pt(spec_pt_in_grip, tool_stage_data, demo_to_exp_tool_transform, arm):
    demo_tool_grip_to_world_transform = get_demo_tool_grip_to_world_transform(tool_stage_data, arm)
    demo_spec_pt_in_world = apply_mat_transform_to_xyz(demo_tool_grip_to_world_transform, spec_pt_in_grip)
    demo_spec_pt_in_world_frame = jut.translation_matrix(demo_spec_pt_in_world)
    plot_spec_pts(np.array([demo_spec_pt_in_world]), (1,0,0,0.5))

    warped_spec_pt_in_world_frame = apply_tps_transform_to_hmat(demo_to_exp_tool_transform, demo_spec_pt_in_world_frame)
    warped_spec_pt_in_world_trans, warped_spec_pt_in_world_rot = juc.hmat_to_trans_rot(warped_spec_pt_in_world_frame)
    plot_spec_pts(np.array([warped_spec_pt_in_world_trans]), (0,1,0,0.5))

# plots a trajectory in rviz; uses RvizWrapper function that displays arrows giving the orientation of the gripper(s)
def plot_traj(xyzs, rgba, quats=None):
    pose_array = juc.array_to_pose_array(np.asarray(xyzs), 'base_footprint', quats)
    Globals.handles.append(Globals.rviz.draw_traj_points(pose_array, rgba = rgba, ns = "multi_item_make_verb_traj_service"))

# plots the special point trajectory; uses the RvizWrapper function that displays points
def plot_spec_pts(xyzs, rgba):
    pose_array = juc.array_to_pose_array(np.asarray(xyzs), 'base_footprint')
    Globals.handles.append(Globals.rviz.draw_traj_points(pose_array, rgba = rgba, ns = "multi_item_make_verb_traj_service"))

# plots the original and warped gripper trajectories; also plots the original and warped special point trajs
def plot_original_and_warped_demo_and_spec_pt(best_demo, warped_demo, spec_pt_xyzs, warped_spec_pt_xyzs, arms_used):
    if arms_used in "lb":
        plot_traj(np.asarray(best_demo["l_gripper_tool_frame"]["position"]),
                  (1,0,0,1),
                  np.asarray(best_demo["l_gripper_tool_frame"]["orientation"]))
        plot_traj(np.asarray(warped_demo["l_gripper_tool_frame"]["position"]),
                  (0,1,0,1),
                  np.asarray(warped_demo["l_gripper_tool_frame"]["orientation"]))
        
    if arms_used in "rb":
        plot_traj(np.asarray(best_demo["r_gripper_tool_frame"]["position"]),
                  (1,0,0,1),
                  np.asarray(best_demo["r_gripper_tool_frame"]["orientation"]))
        plot_traj(np.asarray(warped_demo["r_gripper_tool_frame"]["position"]),
                  (0,1,0,1),
                  np.asarray(warped_demo["r_gripper_tool_frame"]["orientation"]))

    plot_spec_pts(spec_pt_xyzs, (1,0.5,0.5,1))
    plot_spec_pts(warped_spec_pt_xyzs, (0.5,1,0.5,1))

def sorted_values(d):
    return [d[key] for key in sorted(d.keys())]
