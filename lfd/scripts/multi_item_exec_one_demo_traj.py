#!/usr/bin/env python
import argparse
from lfd import multi_item_verbs, make_verb_traj, exec_verb_traj, ik_functions
from verb_msgs.srv import *
import rospy
import numpy as np
import brett2.ros_utils as ru
import roslib; roslib.load_manifest("snazzy_msgs")
from snazzy_msgs.srv import *
import sensor_msgs.msg as sm
from jds_utils.yes_or_no import yes_or_no
from jds_utils.colorize import colorize
import subprocess

def call_and_print(cmd,color='green'):
    print colorize(cmd, color, bold=True)
    subprocess.check_call(cmd, shell=True)

def get_trajectory_request(verb, pc):
    make_req = MakeTrajectoryRequest()
    make_req.verb = verb
    make_req.object_clouds.append(pc)
    return make_req

def do_segmentation(obj_name):
    seg_svc = rospy.ServiceProxy("/interactive_segmentation", ProcessCloud)
    pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)
    pc_tf = ru.transformPointCloud2(pc, exec_verb_traj.Globals.pr2.tf_listener, "base_footprint", pc.header.frame_id)
    print "select the %s" % (obj_name)
    pc_sel = seg_svc.call(ProcessCloudRequest(cloud_in = pc_tf)).cloud_out
    return pc_sel

def do_single(demo_base_name, demo_index, stage_num, prev_demo_index, verb_data_accessor):
    if stage_num == 0:
        do_stage(demo_base_name, demo_index, stage_num, None, None, verb_data_accessor)
    else:
        prev_stage_num = stage_num - 1
        prev_demo_name = demo_base_name + str(prev_demo_index)
        prev_stage_info = verb_data_accessor.get_stage_info(prev_demo_name, prev_stage_num)
        prev_exp_pc = do_segmentation(prev_stage_info.item)

        call_and_print("rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller l_arm_controller")
        print colorize("do the stage involving the %s" % (prev_stage_info.item), color="red", bold=True)
        yes_or_no("type y to continue")
        call_and_print("rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller l_arm_controller")

        do_stage(demo_base_name, demo_index, stage_num, prev_stage_info, np.array([prev_exp_pc]), verb_data_accessor)

def do_stage(demo_base_name, demo_index, stage_num, prev_stage_info, prev_exp_clouds, verb_data_accessor):
    demo_name = demo_base_name + str(demo_index)
    stage_info = verb_data_accessor.get_stage_info(demo_name, stage_num)
    pc_sel = do_segmentation(stage_info.item)
    make_req = get_trajectory_request(stage_info.verb, pc_sel)

    make_resp = make_verb_traj.make_traj_multi_stage(make_req, stage_info, stage_num, prev_stage_info, prev_exp_clouds, verb_data_accessor, use_tps_zrot=False)
    
    yn = yes_or_no("continue?")
    if yn:
        exec_req = ExecTrajectoryRequest()
        exec_req.traj = make_resp.traj
        exec_verb_traj.exec_traj(exec_req, traj_ik_func=ik_functions.do_traj_ik_graph_search, obj_pc=pc_sel, obj_name=stage_info.item)

    # return stage info and object clouds so they can be saved for use in the next stage if necessary
    return (stage_info, make_req.object_clouds)

# get the trajectory for each stage and execute the trajectory
def do_multiple_stages(demo_base_name, stages, verb_data_accessor):
    prev_stage_info, prev_exp_clouds = None, None
    for stage_num, demo_num in enumerate(stages):
        prev_stage_info, prev_exp_clouds = do_stage(demo_base_name, demo_num, stage_num, prev_stage_info, prev_exp_clouds, verb_data_accessor)

def do_globals_setup():
    make_verb_traj.Globals.setup()
    exec_verb_traj.Globals.setup()

def move_pr2_to_start_pos(pr2):
    HEAD_ANGLE = 1.1
    pr2.rgrip.open()
    pr2.lgrip.open()
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')
    pr2.head.set_pan_tilt(0, HEAD_ANGLE)
    pr2.join_all()

def get_exp_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--demo",type=str)
    parser.add_argument("--verb",type=str)
    # comma separated list of demo indicies to use for each stage
    parser.add_argument("--stages",type=str)
    # three values separated by commas: demo index for stage, stage number, and demo index for previous stage
    parser.add_argument("--single",type=str)
    args = parser.parse_args()
    return args

def run_exp(args):
    demo_base_name = args.demo
    print "using demo base", args.demo
    if args.single is not None:
        params = [int(num) for num in args.single.split(',')]
        # make sure there are three values or the stage number is zero
        if len(params) == 3:
            demo_index, stage_num, prev_demo_index = params
        elif len(params) == 2 and params[1] == 0:
            demo_index, stage_num = params
            prev_demo_index = -1
        else:
            raise ValueError("Invalid 'single' argument")
        do_single(demo_base_name, demo_index, stage_num, prev_demo_index, verb_data_accessor)
    elif args.stages is not None:
        if args.stages[0] == 'a':
            stages = [int(args.stages[1:]) for i in range(verb_data_accessor.get_num_stages(demo_base_name))]
        else:
            stages = args.stages
        do_multiple_stages(demo_base_name, stages, verb_data_accessor)

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed": 
        rospy.init_node("multi_item_exec_one_demo", disable_signals=True)
    do_globals_setup()
    args = get_exp_args()
    pr2 = exec_verb_traj.Globals.pr2
    verb_data_accessor = multi_item_verbs.VerbDataAccessor()
    move_pr2_to_start_pos(pr2)

    assert args.demo is not None or args.verb is not None
    # use 'stages' to use all stages, use 'single' for just one stage
    assert (args.stages is not None and args.single is None) or (args.stages is None and args.single is not None)

    if args.demo is not None:
        run_exp(args)
