from lfd import make_verb_traj, multi_item_verbs
import jds_utils.conversions as juc
import jds_utils.transformations as jut
from jds_utils.colorize import colorize
import yaml
from verb_msgs.srv import MakeTrajectoryRequest
import rospy
import numpy as np

def report((x, msg)):
    if x:
        print colorize("Pass", "green")
    else:
        print colorize("Fail: %s" % (msg), "red")

def get_test_params():
    f = open("multi_item_data/multi_item_params.yaml", 'r')
    params = yaml.load(f)
    f.close()
    return params

def xyz_distance(pt1, pt2):
    return (sum([(pt1[i]-pt2[i])**2 for i in range(len(pt1))]))**0.5

def rot_distance(quat1, quat2):
    quat1_mat = jut.quaternion_matrix(quat1) 
    quat2_mat = jut.quaternion_matrix(quat2) 
    diff_mat = np.dot(quat2_mat, np.linalg.inv(quat1_mat))
    diff_xyz, diff_quat = juc.hmat_to_trans_rot(diff_mat)
    return (sum([x**2 for x in jut.euler_from_quaternion(diff_quat)]))**0.5

# What should these values be?
XYZ_TOLERANCE = 0.1
ROT_TOLERANCE = 0.1
        
# returns if the trajectories are approximately the same
def similar_trajectories(traj1, traj2):
    if len(traj1) != len(traj2):
        return False, "trajectory lengths don't match"
    for index, (traj1_pt, traj2_pt) in enumerate(zip(traj1, traj2)):
        traj1_pt_xyz, traj1_pt_rot = juc.hmat_to_trans_rot(traj1_pt)
        traj2_pt_xyz, traj2_pt_rot = juc.hmat_to_trans_rot(traj2_pt)
        xyz_dist = xyz_distance(traj1_pt_xyz, traj2_pt_xyz)
        rot_dist = rot_distance(traj1_pt_rot, traj2_pt_rot)
        if xyz_dist > XYZ_TOLERANCE or rot_dist > ROT_TOLERANCE:
            error_msg = "Incorrect point (index %s): { traj1: %s %s, traj2: %s %s }, { xyz_dist: %s, rot_dist: %s" % (str(index), str(traj1_pt_xyz), str(traj1_pt_rot), str(traj2_pt_xyz), str(traj2_pt_rot), str(xyz_dist), str(rot_dist))
            return False, error_msg
    return True, "matching"

def test_cup_pour_init():
    make_verb_traj.Globals.setup()

# MAKE SURE THAT ROSCORE IS RUNNING FOR THIS TEST, BECAUSE MAKE_VERB_TRAJ DOES PLOTTING FOR RVIZ

# tests that if the target object is the same, then the difference between the demo and experiment special point trajectories are the translation between the the demo and experiment target objects
def test_cup_pour(demo_name, exp_name):
    test_cup_pour_init()

    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test=True)

    current_stage = 1
    gripper_data_key = "r_gripper_tool_frame"

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
    
    # point clouds of tool for demo and experiment
    prev_exp_pc = prev_exp_data["object_clouds"][prev_exp_info.item]["xyz"]
    cur_exp_pc = cur_exp_data["object_clouds"][cur_exp_info.item]["xyz"]

    # calculate the transformation from the world frame to the gripper frame
    prev_exp_gripper_pos = prev_exp_data[gripper_data_key]["position"][-1]
    prev_exp_gripper_orien = prev_exp_data[gripper_data_key]["orientation"][-1]
    prev_world_to_gripper_trans = np.linalg.inv(juc.trans_rot_to_hmat(prev_exp_gripper_pos, prev_exp_gripper_orien))
    gripper_frame_trans = make_verb_traj.make_to_gripper_frame_hmat(prev_world_to_gripper_trans)

    warped_traj_resp = make_verb_traj.make_traj_multi_stage_do_work(cur_demo_info, [cur_exp_pc], None, current_stage, prev_demo_info, [prev_exp_pc], verb_data_accessor, gripper_frame_trans)

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
    cur_exp_traj_as_mats = [juc.pose_to_hmat(pose) for pose in result_traj.r_gripper_poses.poses]

    report(similar_trajectories(expected_gripper_traj, cur_exp_traj_as_mats))

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed":
        rospy.init_node('test_multi_item',disable_signals=True)
    test_cup_pour("pour-green0-blue0", "pour-green1-blue0")
