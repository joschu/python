import rospy
import sys
from lfd import exec_verb_traj, multi_item_make_verb_traj, multi_item_verbs, ik_functions
from verb_msgs.srv import ExecTrajectoryRequest
from jds_utils.yes_or_no import yes_or_no
import yaml
import os.path as osp
import os
import argparse

EMPTY_MOVE_PARAM_FILE = osp.join(osp.dirname(osp.abspath(__file__)), "multi_item/multi_item_params/empty_move_params.yaml")
TEST_DATA_DIR = "multi_item/empty_move_data"

def get_test_params():
    f = open(EMPTY_MOVE_PARAM_FILE, 'r')
    params = yaml.load(f)
    f.close()
    return params

def set_table_params():
    params = get_test_params()
    table_bounds = params["table_bounds"]
    rospy.set_param("table_height", float(table_bounds[-1]))
    rospy.set_param("table_bounds", " ".join([str(x) for x in table_bounds]))

def move_to_start_pos(pr2):
    pr2.torso.go_up()
    pr2.rgrip.open()
    pr2.lgrip.open()
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')
    pr2.join_all()
    
def pour_empty_move_init():
    set_table_params()
    exec_verb_traj.Globals.setup()
    multi_item_make_verb_traj.Globals.setup()
    move_to_start_pos(exec_verb_traj.Globals.pr2)

# makes the PR2 go through motions using fake data
def do_empty_move(demo_name, exp_name, test_dir_name):
    pour_empty_move_init()

    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test_info_dir=osp.join("test", TEST_DATA_DIR, test_dir_name))

    for current_stage in xrange(verb_data_accessor.get_num_stages(demo_name)):
        # info and data for previous stage
        if current_stage > 0:
            prev_demo_info = verb_data_accessor.get_stage_info(demo_name, current_stage-1)
            prev_exp_info = verb_data_accessor.get_stage_info(exp_name, current_stage-1)
            prev_exp_data = verb_data_accessor.get_demo_stage_data(prev_exp_info.stage_name)
            prev_exp_pc = prev_exp_data["object_cloud"][prev_exp_info.item]["xyz"]
        else:
            prev_demo_info, prev_exp_info, prev_exp_data, prev_exp_pc = None, None, None, None

        # info and data for current stage
        cur_demo_info = verb_data_accessor.get_stage_info(demo_name, current_stage)
        cur_exp_info = verb_data_accessor.get_stage_info(exp_name, current_stage)
        cur_exp_data = verb_data_accessor.get_demo_stage_data(cur_exp_info.stage_name)
        cur_exp_pc = cur_exp_data["object_cloud"][cur_exp_info.item]["xyz"]

        warped_traj_resp = multi_item_make_verb_traj.make_traj_multi_stage_do_work(cur_demo_info, cur_exp_pc,
                                                                                   "base_footprint", current_stage,
                                                                                   prev_demo_info, prev_exp_pc,
                                                                                   verb_data_accessor, transform_type="tps_zrot")

        yn = yes_or_no("continue?")
        if yn:
            traj = warped_traj_resp.traj
            exec_verb_traj.exec_traj_do_work(traj.l_gripper_poses.poses, traj.l_gripper_angles,
                                             traj.r_gripper_poses.poses, traj.r_gripper_angles,
                                             traj_ik_func=ik_functions.do_traj_ik_graph_search,
                                             obj_cloud_xyz=cur_exp_pc, obj_name=cur_demo_info.item)

        if current_stage < verb_data_accessor.get_num_stages(demo_name)-1:
            yn = yes_or_no("continue to next stage?")
            if not yn:
                break

def get_test_demos():
    abs_test_dir = osp.join(osp.dirname(osp.abspath(__file__)), TEST_DATA_DIR)
    test_demo_dirs = os.listdir(abs_test_dir)
    test_demos = {}
    for test_demo_dir in test_demo_dirs:
        verb_data_accessor = multi_item_verbs.VerbDataAccessor(osp.join(abs_test_dir, test_demo_dir))
        all_demo_info = verb_data_accessor.get_all_demo_info()
        verb = all_demo_info[all_demo_info.keys()[0]]["verb"]
        test_demos[verb] = (test_demo_dir, all_demo_info.keys())
    return test_demos

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("verb")
    parser.add_argument("item1")
    parser.add_argument("item2")
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed":
        rospy.init_node("test_multi_item_empty_move",disable_signals=True)

    if len(sys.argv) == 1:
        print get_test_demos()

    args = get_args()
    all_test_demos = get_test_demos()

    demo_part, exp_part = None, None
    test_demo_dir, verb_demos = all_test_demos[args.verb]
    for demo_name in verb_demos:
        if demo_name.find(args.item1) != -1:
            demo_part = demo_name
        if demo_name.find(args.item2) != -1:
            exp_part = demo_name

    assert demo_part is not None and exp_part is not None, all_test_demos[args.verb]
    print "Testing with demo %s and experiment %s from directory %s" % (demo_part, exp_part, test_demo_dir)
    do_empty_move(demo_part, exp_part, test_demo_dir)
