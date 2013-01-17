import rospy
from lfd import multi_item_make_verb_traj, multi_item_verbs
import jds_utils.conversions as juc
import jds_utils.transformations as jut
from jds_utils.colorize import colorize
import yaml
import numpy as np
import os.path as osp

TRANSLATION_PARAM_FILE = osp.join(osp.dirname(__file__), "multi_item/multi_item_params/translation_params.yaml")

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

def rot_distance(quat1, quat2):
    quat1_mat = jut.quaternion_matrix(quat1) 
    quat2_mat = jut.quaternion_matrix(quat2) 
    diff_mat = np.dot(quat2_mat, np.linalg.inv(quat1_mat))
    diff_xyz, diff_quat = juc.hmat_to_trans_rot(diff_mat)
    return euclidean_dist(diff_quat, [0, 0, 0, 1])

# What should these values be?
XYZ_TOLERANCE = 0.025
ROT_TOLERANCE = 0.2
        
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

def test_cup_pour_init():
    multi_item_make_verb_traj.Globals.setup()

# MAKE SURE THAT ROSCORE IS RUNNING FOR THIS TEST, BECAUSE MULTI_ITEM_MAKE_VERB_TRAJ DOES PLOTTING FOR RVIZ

# tests that if the target object is the same, then the difference between the demo and experiment special point trajectories are the translation between the the demo and experiment target objects
def test_translation(demo_name, exp_name, data_dir):
    test_cup_pour_init()

    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test_info_dir=("test/multi_item/multi_item_data/"+data_dir))

    current_stage = 1

    # info and data for previous stage
    prev_demo_info = verb_data_accessor.get_stage_info(demo_name, current_stage-1)
    prev_demo_data = verb_data_accessor.get_demo_data(prev_demo_info.stage_name)
    prev_exp_info = verb_data_accessor.get_stage_info(exp_name, current_stage-1)
    prev_exp_data = verb_data_accessor.get_demo_data(prev_exp_info.stage_name)

    # info and data for current stage
    cur_demo_info = verb_data_accessor.get_stage_info(demo_name, current_stage)
    cur_demo_data = verb_data_accessor.get_demo_data(cur_demo_info.stage_name)
    cur_exp_info = verb_data_accessor.get_stage_info(exp_name, current_stage)
    cur_exp_data = verb_data_accessor.get_demo_data(cur_exp_info.stage_name)
    
    gripper_data_key = "%s_gripper_tool_frame" % cur_demo_info.arms_used

    # point clouds of tool for demo and experiment
    prev_exp_pc = prev_exp_data["object_cloud"][prev_exp_info.item]["xyz"]
    cur_exp_pc = cur_exp_data["object_cloud"][cur_exp_info.item]["xyz"]

    # calculate the transformation from the world frame to the gripper frame
    prev_exp_gripper_pos = prev_exp_data[gripper_data_key]["position"][-1]
    prev_exp_gripper_orien = prev_exp_data[gripper_data_key]["orientation"][-1]
    prev_world_to_gripper_trans = np.linalg.inv(juc.trans_rot_to_hmat(prev_exp_gripper_pos, prev_exp_gripper_orien))

    gripper_frame_trans = multi_item_make_verb_traj.make_to_gripper_frame_hmat(prev_world_to_gripper_trans)

    warped_traj_resp = multi_item_make_verb_traj.make_traj_multi_stage_do_work(cur_demo_info, cur_exp_pc, None, current_stage, prev_demo_info, prev_exp_pc, verb_data_accessor, to_gripper_frame_func=gripper_frame_trans, transform_type="tps")

    # get the actual transformation between the old and new target objects (just a translation for this test)
    params = get_test_params()
    translation = params['translation']
    actual_target_translation_matrix = jut.translation_matrix(translation)

    # get the demo special point trajectory
    cur_demo_gripper_traj_xyzs = cur_demo_data[gripper_data_key]["position"]
    cur_demo_gripper_traj_oriens = cur_demo_data[gripper_data_key]["orientation"]
    cur_demo_gripper_traj_mats = [juc.trans_rot_to_hmat(trans, orien) for (trans, orien) in zip(cur_demo_gripper_traj_xyzs, cur_demo_gripper_traj_oriens)]
    prev_demo_spec_pt_translation = jut.translation_matrix(np.array(prev_demo_info.special_point))
    cur_demo_spec_pt_traj_as_mats = [np.dot(traj_mat, prev_demo_spec_pt_translation) for traj_mat in cur_demo_gripper_traj_mats]

    # get the expected experiment special point trajectory
    expected_spec_pt_traj = [np.dot(actual_target_translation_matrix, traj_mat) for traj_mat in cur_demo_spec_pt_traj_as_mats]

    # get the expected experiment gripper trajectory
    prev_exp_spec_pt_translation = jut.translation_matrix(np.array(prev_exp_info.special_point))
    inv_cur_exp_spec_pt_translation = np.linalg.inv(prev_exp_spec_pt_translation)
    expected_gripper_traj = [np.dot(traj_mat, inv_cur_exp_spec_pt_translation) for traj_mat in expected_spec_pt_traj]

    # compare the expected new special point trajectory to the result of make_traj_multi_stage
    result_traj = warped_traj_resp.traj
    cur_exp_traj_as_mats = [juc.pose_to_hmat(pose) for pose in result_traj.l_gripper_poses.poses]

    report(similar_trajectories(expected_gripper_traj, cur_exp_traj_as_mats))

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed":
        rospy.init_node("test_multi_item_translation",disable_signals=True)
    test_translation("pour-yellow0-blue0", "pour-yellow1-blue0", "pour_yellow_blue_l_l")
