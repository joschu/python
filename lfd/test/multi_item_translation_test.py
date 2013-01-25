import rospy
from lfd import multi_item_make_verb_traj, multi_item_verbs
import jds_utils.conversions as juc
import jds_utils.transformations as jut
from jds_utils.colorize import colorize
import yaml
import numpy as np
import os.path as osp

# Tests that warping does the right thing for a simple translation of the target (everything else stays the same)

TEST_DATA_DIR = "multi_item/translation_test_data"
TRANSLATION_PARAM_FILE = osp.join(osp.dirname(__file__), TEST_DATA_DIR, "translation_params.yaml")

def report((x, msg)):
    if x:
        print colorize("Pass", "green")
    else:
        print colorize("Fail: %s" % (msg), "red")

def get_test_params():
    f = open(TRANSLATION_PARAM_FILE, 'r')
    params = yaml.load(f)
    f.close()
    return params

def euclidean_dist(pt1, pt2):
    return (sum([(pt1[i]-pt2[i])**2 for i in range(len(pt1))]))**0.5

# finds the difference between the quats and then returns euclidean distance from the identity quat
def rot_distance(quat1, quat2):
    quat1_mat = jut.quaternion_matrix(quat1) 
    quat2_mat = jut.quaternion_matrix(quat2) 
    diff_mat = np.dot(quat2_mat, np.linalg.inv(quat1_mat))
    diff_xyz, diff_quat = juc.hmat_to_trans_rot(diff_mat)
    return euclidean_dist(diff_quat, [0, 0, 0, 1])

# What should these values be?
XYZ_TOLERANCE = 0.02
ROT_TOLERANCE = 0.1
        
# returns if the trajectories are approximately the same
def similar_trajectories(traj1, traj2):
    if len(traj1) != len(traj2):
        return False, "trajectory lengths don't match"
    for index, (traj1_pt, traj2_pt) in enumerate(zip(traj1, traj2)):
        traj1_pt_xyz, traj1_pt_rot = juc.hmat_to_trans_rot(traj1_pt)
        traj2_pt_xyz, traj2_pt_rot = juc.hmat_to_trans_rot(traj2_pt)
        xyz_dist = euclidean_dist(traj1_pt_xyz, traj2_pt_xyz)
        rot_dist = rot_distance(traj1_pt_rot, traj2_pt_rot)
        if xyz_dist > XYZ_TOLERANCE or rot_dist > ROT_TOLERANCE:
            error_msg = "Incorrect point (index %i): { traj1: %s %s, traj2: %s %s }, { xyz_dist: %f, rot_dist: %f }" % (index, str(traj1_pt_xyz), str(traj1_pt_rot), str(traj2_pt_xyz), str(traj2_pt_rot), xyz_dist, rot_dist)
            return False, error_msg
    return True, "matching"

# get the demo special point trajectory
def get_demo_spec_pt_traj_mats(demo_target_data, gripper_data_key):
    demo_grip_traj_xyzs = demo_target_data[gripper_data_key]["position"]
    demo_grip_traj_oriens = demo_target_data[gripper_data_key]["orientation"]
    demo_grip_traj_mats = [juc.trans_rot_to_hmat(trans, orien) for (trans, orien) in zip(demo_grip_traj_xyzs, demo_grip_traj_oriens)]
    demo_tool_spec_pt_translation = jut.translation_matrix(np.array(demo_tool_info.special_point))
    demo_spec_pt_traj_mats = [np.dot(traj_mat, demo_tool_spec_pt_translation) for traj_mat in demo_grip_traj_mats]
    return demo_spec_pt_traj_mats

# translates the expected special point trajectory to a gripper trajectory
def get_expected_gripper_traj(special_point, expected_spec_pt_traj):
    exp_tool_spec_pt_translation = jut.translation_matrix(np.array(exp_tool_info.special_point))
    inv_exp_tool_spec_pt_translation = np.linalg.inv(exp_tool_spec_pt_translation)
    expected_gripper_traj = [np.dot(traj_mat, inv_exp_tool_spec_pt_translation) for traj_mat in expected_spec_pt_traj]
    return expected_gripper_traj

def get_world_to_grip_exp_transform(exp_tool_data, gripper_data_key):
    exp_gripper_pos = exp_tool_data[gripper_data_key]["position"][-1]
    exp_gripper_orien = exp_tool_data[gripper_data_key]["orientation"][-1]
    world_to_grip_transform = np.linalg.inv(juc.trans_rot_to_hmat(exp_gripper_pos, exp_gripper_orien))
    return world_to_grip_transform

def translation_test_init():
    multi_item_make_verb_traj.Globals.setup()

# MAKE SURE THAT ROSCORE IS RUNNING FOR THIS TEST, BECAUSE MULTI_ITEM_MAKE_VERB_TRAJ DOES PLOTTING FOR RVIZ

def test_translation(demo_name, exp_name, data_dir):
    translation_test_init()

    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test_info_dir=osp.join("test", TEST_DATA_DIR, data_dir))

    current_stage = 1

    # info and data for tool stage
    demo_tool_info = verb_data_accessor.get_stage_info(demo_name, current_stage-1)
    demo_tool_data = verb_data_accessor.get_demo_stage_data(demo_tool_info.stage_name)
    exp_tool_info = verb_data_accessor.get_stage_info(exp_name, current_stage-1)
    exp_tool_data = verb_data_accessor.get_demo_stage_data(exp_tool_info.stage_name)

    # info and data for target stage
    demo_target_info = verb_data_accessor.get_stage_info(demo_name, current_stage)
    demo_target_data = verb_data_accessor.get_demo_stage_data(demo_target_info.stage_name)
    exp_target_info = verb_data_accessor.get_stage_info(exp_name, current_stage)
    exp_target_data = verb_data_accessor.get_demo_stage_data(exp_target_info.stage_name)
    
    gripper_data_key = "%s_gripper_tool_frame" % demo_target_info.arms_used

    # point clouds of tool for demo and experiment
    exp_tool_pc = exp_tool_data["object_cloud"][exp_tool_info.item]["xyz"]
    exp_target_pc = exp_target_data["object_cloud"][exp_target_info.item]["xyz"]

    # calculate the transformation from the world frame to the gripper frame in the experiment scene
    world_to_grip_transform = get_world_to_grip_exp_transform(exp_tool_data, gripper_data_key)
    world_to_grip_transform_func = multi_item_make_verb_traj.make_world_to_grip_transform_hmat(world_to_grip_transform)

    warped_traj_resp = multi_item_make_verb_traj.make_traj_multi_stage_do_work(demo_name, exp_target_pc,
                                                                               None, current_stage,
                                                                               demo_tool_info, exp_tool_pc,
                                                                               verb_data_accessor, world_to_grip_transform_func,
                                                                               "tps")

    # assuming that the arms_used for the target stage is 'l' or 'r'
    if demo_target_info.arms_used == 'l':
        warped_grip_traj_mats = [juc.pose_to_hmat(pose) for pose in warped_traj_resp.traj.l_gripper_poses.poses]
    elif demo_target_info.arms_used == 'r':
        warped_grip_traj_mats = [juc.pose_to_hmat(pose) for pose in warped_traj_resp.traj.r_gripper_poses.poses]

    # get the manually measured transformation between the old and new target objects (just a translation for this test)
    params = get_test_params()
    actual_target_translation = jut.translation_matrix(params["translation"])

    # find the expected warped gripper trajectory using the manual translation measurement
    demo_spec_pt_traj_mats = get_demo_spec_pt_traj_mats(demo_target_data, gripper_data_key)
    expected_spec_pt_traj = [np.dot(actual_target_translation, traj_mat) for traj_mat in demo_spec_pt_traj_mats]
    expected_gripper_traj = get_expected_gripper_traj(exp_tool_info.special_point, expected_spec_pt_traj)

    result = similar_trajectories(expected_gripper_traj, warped_grip_traj_mats)
    report(result)

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed":
        rospy.init_node("test_multi_item_translation",disable_signals=True)
    test_translation("place-cup-bowl0", "place-cup-bowltranslated0", "place_cup_bowl_l_l")
