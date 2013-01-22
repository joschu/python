import rospy
from brett2.ros_utils import RvizWrapper, Marker, pc2xyzrgb
import brett2.ros_utils as ru
import h5py
from lfd import warping, registration, tps
import roslib
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import MakeTrajectoryRequest, MakeTrajectoryResponse
from numpy import asarray
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
        Globals.rviz = RvizWrapper.create()

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
    
# returns appends 1 to the end of the point so it can be applied to a 4x4 matrix
def get_homog_coord(point):
    homog = np.ones(4)
    homog[:3] = point
    return homog

# gets the 3d point out of a homogeneous coordinate
def get_array3(homog):
    return homog[:3]
    
# applies a transformation specified by a 4x4 matrix to a 3d point
# transform is a 4x4 matrix (np.array) and point is a 3-element vector (np.array)
def apply_mat_transform_to_xyz(transform, point):
    applied = np.dot(transform, get_homog_coord(point))
    return get_array3(applied)

# applies tps transformation to a frame specified by hmat
# if tps_transform is None, the hmat is returned
def apply_tps_transform_to_hmat(tps_transform, hmat):
    if tps_transform is None:
        return hmat
    xyz, quat = juc.hmat_to_trans_rot(hmat)
    warped_xyz, warped_mat = tps_transform.transform_frames(np.array([xyz]), np.array([juc.quat2mat(quat)]))
    return juc.trans_rot_to_hmat(warped_xyz[0], juc.mat2quat(warped_mat[0]))

# transforms point cloud in base frame to point cloud in gripper frame using tf_listener
def to_gripper_frame_tf_listener(pc, gripper_frame_name):
    return ru.transform_points(pc, ru.get_tf_listener(), gripper_frame_name, "base_footprint")

# makes a function that transforms point cloud in base frame to point cloud in gripper frame using supplied rigid transformation
def make_to_gripper_frame_hmat(transformation):
    def to_gripper_frame_hmat(pc, gripper_frame_name):
        return np.array([apply_mat_transform_to_xyz(transformation, point) for point in pc])
    return to_gripper_frame_hmat

def print_hmat_info(hmat, title):
    #trans, rot = juc.hmat_to_trans_rot(hmat)
    #print trans, jut.euler_from_quaternion(rot)
    print title
    print hmat

# transform point cloud in base frame to gripper frame
# use the last pose of the gripper in the stage to figure out the point cloud of the tool in the gripper frame when the tool was grabbed
def get_prev_demo_pc_in_gripper_frame(prev_stage_data, prev_stage_info, arm):
    gripper_data_key = "%s_gripper_tool_frame" % (arm)
    prev_demo_gripper_pos = prev_stage_data[gripper_data_key]["position"][-1]
    prev_demo_gripper_orien = prev_stage_data[gripper_data_key]["orientation"][-1]
    prev_demo_gripper_to_base_transform = juc.trans_rot_to_hmat(prev_demo_gripper_pos, prev_demo_gripper_orien)
    prev_demo_base_to_gripper_transform = np.linalg.inv(prev_demo_gripper_to_base_transform)
    prev_demo_pc = prev_stage_data["object_cloud"][prev_stage_info.item]["xyz"]
    prev_demo_pc_down = voxel_downsample(prev_demo_pc, .02)
    prev_demo_pc_in_gripper_frame = np.array([apply_mat_transform_to_xyz(prev_demo_base_to_gripper_transform, point) for point in prev_demo_pc_down])
    return prev_demo_pc_in_gripper_frame

# transforms a cloud in the global frame to one in the gripper frame using to_gripper_frame_func
# gripper data key is {l,r}_gripper_tool_frame
# if to_gripper_frame_func is None, then tf is used to do the transformation
def get_prev_exp_pc_in_gripper_frame(prev_exp_cloud, gripper_data_key, to_gripper_frame_func):
    prev_exp_pc_down = voxel_downsample(prev_exp_cloud, .02)
    if to_gripper_frame_func is None:
        prev_exp_pc_in_gripper_frame = to_gripper_frame_tf_listener(prev_exp_pc_down, gripper_data_key)
    else:
        prev_exp_pc_in_gripper_frame = to_gripper_frame_func(prev_exp_pc_down, gripper_data_key)
    return prev_exp_pc_in_gripper_frame

# gets a transformation as a 4x4 matrix corresponding to the translation specified by special_point
# if special_point is None, then the identity is returned
def get_special_point_translation(special_point):
    if special_point is None:
        # don't do a special point translation if there is no specified special point
        special_point_translation = np.identity(4)
    else:
        special_point_translation = jut.translation_matrix(np.array(special_point))
    return special_point_translation

# gets the warping transformation for a tool in a gripper (arm specifies which gripper, and it should be {l,r})
# prev_stage_data and prev_stage_info specify the demo cloud for the tool, and prev_exp_cloud is the experiment cloud
# to_gripper_frame_func is a function that transforms the experiment cloud to the gripper frame; if it is None, then tf is used
def get_prev_demo_to_exp_grip_transform(prev_stage_data, prev_stage_info, prev_exp_cloud, arm, to_gripper_frame_func, transform_type):
    gripper_data_key = "%s_gripper_tool_frame" % (arm)
    prev_demo_pc_in_gripper_frame = get_prev_demo_pc_in_gripper_frame(prev_stage_data, prev_stage_info, arm)
    prev_exp_pc_in_gripper_frame = get_prev_exp_pc_in_gripper_frame(prev_exp_cloud, gripper_data_key, to_gripper_frame_func)
    prev_demo_to_exp_grip_transform = get_warping_transform(prev_demo_pc_in_gripper_frame,
                                                            prev_exp_pc_in_gripper_frame,
                                                            transform_type)
    return prev_demo_to_exp_grip_transform

# returns the demo gripper trajectory for the current stage as an array of matrices
def get_cur_demo_gripper_traj_mats(verb_stage_data, arm):
    gripper_data_key = "%s_gripper_tool_frame" % (arm)
    cur_demo_gripper_traj_xyzs = verb_stage_data[gripper_data_key]["position"]
    cur_demo_gripper_traj_oriens = verb_stage_data[gripper_data_key]["orientation"]
    cur_demo_gripper_traj_mats = [juc.trans_rot_to_hmat(trans, orien) for (trans, orien) in zip(cur_demo_gripper_traj_xyzs, cur_demo_gripper_traj_oriens)]
    return cur_demo_gripper_traj_mats

# gets the warping transform for the target object of the current stage
def get_cur_demo_to_exp_transform(cur_demo_cloud, cur_exp_cloud, transform_type):
    x_nd = voxel_downsample(cur_demo_cloud, .02)
    y_md = voxel_downsample(cur_exp_cloud, .02)
    cur_demo_to_exp_transform = get_warping_transform(x_nd, y_md, transform_type)
    return cur_demo_to_exp_transform

# applies the special point translation to a gripper trajectory
def get_cur_demo_spec_pt_traj_mats(cur_demo_gripper_traj_mats, special_point_translation):
    cur_demo_spec_pt_traj_mats = [np.dot(gripper_mat, special_point_translation) for gripper_mat in cur_demo_gripper_traj_mats]
    return cur_demo_spec_pt_traj_mats

# apply the warping transformation to the special point trajectory
def get_cur_exp_spec_pt_traj_mats(cur_demo_spec_pt_traj_mats, cur_demo_to_exp_transform):
    cur_demo_spec_pt_traj_xyzs, cur_demo_spec_pt_traj_oriens = [], []
    for cur_demo_spec_pt_traj_mat in cur_demo_spec_pt_traj_mats:
        cur_demo_spec_pt_traj_xyz, cur_demo_spec_pt_traj_orien = juc.hmat_to_trans_rot(cur_demo_spec_pt_traj_mat)
        cur_demo_spec_pt_traj_xyzs.append(cur_demo_spec_pt_traj_xyz)
        cur_demo_spec_pt_traj_oriens.append(juc.quat2mat(cur_demo_spec_pt_traj_orien))
    cur_exp_spec_pt_traj_xyzs, cur_exp_spec_pt_traj_oriens = cur_demo_to_exp_transform.transform_frames(np.array(cur_demo_spec_pt_traj_xyzs), np.array(cur_demo_spec_pt_traj_oriens))
    cur_exp_spec_pt_traj_mats = [juc.trans_rot_to_hmat(cur_exp_spec_pt_traj_xyz, juc.mat2quat(cur_exp_spec_pt_traj_orien)) for cur_exp_spec_pt_traj_xyz, cur_exp_spec_pt_traj_orien in zip(cur_exp_spec_pt_traj_xyzs, cur_exp_spec_pt_traj_oriens)]
    return cur_exp_spec_pt_traj_mats

# gets the new gripper trajectory from the warped special point trajectory
def get_cur_exp_gripper_traj_mats(cur_exp_spec_pt_traj_mats, prev_demo_to_exp_grip_transform, special_point_translation):
    # transformation from the new special point to the gripper frame
    cur_exp_inv_spec_pt_transform = np.linalg.inv(apply_tps_transform_to_hmat(prev_demo_to_exp_grip_transform, special_point_translation))
    print_hmat_info(cur_exp_inv_spec_pt_transform, "cur_exp_inv_spec_pt_transform")
    # transform the warped special point trajectory back to a gripper trajectory in the experiment
    cur_exp_gripper_traj_mats = [np.dot(spec_pt_mat, cur_exp_inv_spec_pt_transform) for spec_pt_mat in cur_exp_spec_pt_traj_mats]
    return cur_exp_gripper_traj_mats

def set_traj_fields_for_response(warped_stage_data, traj, arm, frame_id):
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
def make_traj_multi_stage(req, current_stage_info, stage_num, prev_stage_info, prev_exp_cloud_pc2, verb_data_accessor, to_gripper_frame_func=None, transform_type="tps"):
    assert isinstance(req, MakeTrajectoryRequest)
    assert len(req.object_clouds) == 1
    prev_exp_cloud = None if stage_num == 0 else pc2xyzrgb(prev_exp_cloud_pc2)[0]
    cur_exp_cloud = pc2xyzrgb(req.object_clouds[0])[0]
    clouds_frame_id = req.object_clouds[0].header.frame_id
    return make_traj_multi_stage_do_work(current_stage_info, cur_exp_cloud, clouds_frame_id, stage_num, prev_stage_info, prev_exp_cloud, verb_data_accessor, to_gripper_frame_func, transform_type)

# the clouds here have already been processed into lists of xyzs
# make trajectory for a certain stage of a task
# current_stage_info is the demo information to use
# stage_num is the current task stage number; previous information is unused if this is zero
# prev_exp_clouds has the point cloud of the object from the previous stage in the gripper frame
# 'prev' and 'cur' is for the previous and current stages; 'demo' and 'exp' are for demonstration and new experiment situations, respectively
# to_gripper_frame_func transforms a point cloud in base frame to point cloud in gripper frame; if it is None, then it takes the transformation from the robot
def make_traj_multi_stage_do_work(current_stage_info, cur_exp_cloud, frame_id, stage_num, prev_stage_info, prev_exp_cloud, verb_data_accessor, to_gripper_frame_func=None, transform_type="tps"):

    arms_used = current_stage_info.arms_used

    # make sure this is the first stage or there is no tool or there is a tool and only one arm is used
    assert stage_num == 0 or (prev_stage_info.item == "none") or (arms_used in ['r', 'l'])

    if stage_num == 0 or prev_stage_info.item == "none":
        # don't do any extra transformation for the first stage
        prev_demo_to_exp_grip_transform = None
        # no special point translation for first stage since no tool yet
        special_point_translation = np.identity(4)
    else:
        prev_verb_stage_data = verb_data_accessor.get_demo_stage_data(prev_stage_info.stage_name)
        # make sure that the tool stage only uses one arm (the one with the tool)
        prev_demo_to_exp_grip_transform = get_prev_demo_to_exp_grip_transform(prev_verb_stage_data, prev_stage_info,
                                                                              prev_exp_cloud, arms_used,
                                                                              to_gripper_frame_func, transform_type)
        special_point_translation = get_special_point_translation(prev_stage_info.special_point)

    cur_verb_stage_data = verb_data_accessor.get_demo_stage_data(current_stage_info.stage_name)
    warped_stage_data = group_to_dict(cur_verb_stage_data) # deep copy it

    resp = MakeTrajectoryResponse()
    traj = resp.traj
    traj.arms_used = arms_used

    # find the target transformation for the experiment scene
    cur_demo_to_exp_transform = get_cur_demo_to_exp_transform(cur_verb_stage_data["object_cloud"][current_stage_info.item]["xyz"],
                                                              cur_exp_cloud,
                                                              transform_type)

    arms_used_list = ['r', 'l'] if arms_used == 'b' else [arms_used]
    for arm in arms_used_list:
        gripper_data_key = "%s_gripper_tool_frame" % (arm)
        
        # get the gripper trajectory before the target transformation
        cur_demo_gripper_traj_mats = get_cur_demo_gripper_traj_mats(cur_verb_stage_data, arm)

        # get the special point trajectory before the target transformation
        cur_demo_spec_pt_traj_mats = get_cur_demo_spec_pt_traj_mats(cur_demo_gripper_traj_mats, special_point_translation)

        # apply the target warping transformation to the special point trajectory
        cur_exp_spec_pt_traj_mats = get_cur_exp_spec_pt_traj_mats(cur_demo_spec_pt_traj_mats, cur_demo_to_exp_transform)

        # get the warped trajectory for the gripper
        cur_exp_gripper_traj_mats = get_cur_exp_gripper_traj_mats(cur_exp_spec_pt_traj_mats,
                                                                  prev_demo_to_exp_grip_transform,
                                                                  special_point_translation)

        warped_transs, warped_rots = juc.hmats_to_transs_rots(cur_exp_gripper_traj_mats)
        warped_stage_data[gripper_data_key]["position"] = warped_transs
        warped_stage_data[gripper_data_key]["orientation"] = warped_rots

        set_traj_fields_for_response(warped_stage_data, traj, arm, frame_id)

        # save the demo special point traj for plotting
        demo_spec_pt_xyzs, exp_spec_pt_xyzs = [], []
        if stage_num > 0:
            demo_spec_pt_xyzs = juc.hmats_to_transs_rots(cur_demo_spec_pt_traj_mats)[0]
            exp_spec_pt_xyzs = juc.hmats_to_transs_rots(cur_exp_spec_pt_traj_mats)[0]

    Globals.handles = []
    plot_original_and_warped_demo_and_spec_pt(cur_verb_stage_data, warped_stage_data,
                                              demo_spec_pt_xyzs, exp_spec_pt_xyzs,
                                              arms_used, traj)

    return resp

def plot_traj(xyzs, rgba, quats=None):
    pose_array = juc.array_to_pose_array(asarray(xyzs), 'base_footprint', quats)
    Globals.handles.append(Globals.rviz.draw_traj_points(pose_array, rgba = rgba, ns = "multi_item_make_verb_traj_service"))

def plot_spec_pts(xyzs, rgba):
    pose_array = juc.array_to_pose_array(asarray(xyzs), 'base_footprint')
    Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = rgba, ns = "multi_item_make_verb_traj_service"))

def plot_original_and_warped_demo_and_spec_pt(best_demo, warped_demo, spec_pt_xyzs, warped_spec_pt_xyzs, arms_used, traj):
    if arms_used in "lb":
        plot_traj(asarray(best_demo["l_gripper_tool_frame"]["position"]),
                  (1,0,0,1),
                  asarray(best_demo["l_gripper_tool_frame"]["orientation"]))
        plot_traj(asarray(warped_demo["l_gripper_tool_frame"]["position"]),
                  (0,1,0,1),
                  asarray(warped_demo["l_gripper_tool_frame"]["orientation"]))
        
    if arms_used in "rb":
        plot_traj(asarray(best_demo["r_gripper_tool_frame"]["position"]),
                  (1,0,0,1),
                  asarray(best_demo["r_gripper_tool_frame"]["orientation"]))
        plot_traj(asarray(warped_demo["r_gripper_tool_frame"]["position"]),
                  (0,1,0,1),
                  asarray(warped_demo["r_gripper_tool_frame"]["orientation"]))

    plot_spec_pts(spec_pt_xyzs, (1,0.5,0.5,1))
    plot_spec_pts(warped_spec_pt_xyzs, (0.5,1,0.5,1))

def sorted_values(d):
    return [d[key] for key in sorted(d.keys())]
