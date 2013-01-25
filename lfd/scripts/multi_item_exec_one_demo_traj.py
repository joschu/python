#!/usr/bin/env python
import rospy
import argparse
from brett2.ros_utils import pc2xyzrgb, xyz2pc, RvizWrapper, Marker
from lfd import multi_item_verbs, multi_item_make_verb_traj, exec_verb_traj, ik_functions
from verb_msgs.srv import *
import numpy as np
from brett2 import ros_utils
import roslib; roslib.load_manifest("snazzy_msgs")
from snazzy_msgs.srv import *
import sensor_msgs.msg as sm
from jds_utils.yes_or_no import yes_or_no
from jds_utils.colorize import colorize
import subprocess
from lfd import scene_diff
from rope_vision import rope_initialization as ri
from jds_image_proc.clouds import voxel_downsample
from jds_utils import conversions as juc

class Globals:
    handles = []
    rviz = None
    isinstance(rviz, ros_utils.RvizWrapper)
    
    def __init__(self): raise

    @staticmethod
    def setup():
        Globals.rviz = ros_utils.RvizWrapper.create()

def call_and_print(cmd, color='green'):
    print colorize(cmd, color, bold=True)
    subprocess.check_call(cmd, shell=True)

def get_trajectory_request(verb, pc):
    make_req = MakeTrajectoryRequest()
    make_req.verb = verb
    make_req.object_clouds.append(pc)
    return make_req

# filters point cloud and displays the point cloud
def filter_pc2(cloud_pc2):
    cloud_xyz = (pc2xyzrgb(cloud_pc2)[0]).reshape(-1,3)
    cloud_xyz_down = voxel_downsample(cloud_xyz, .02)
    graph = ri.points_to_graph(cloud_xyz_down, .03)
    cc = ri.largest_connected_component(graph)
    good_xyzs = np.array([graph.node[node_id]["xyz"] for node_id in cc.nodes()])
    pose_array = juc.array_to_pose_array(good_xyzs, "base_footprint")
    Globals.handles.append(Globals.rviz.draw_curve(pose_array, rgba = (1,1,0,1), type=Marker.CUBE_LIST, width=.001, ns="segmentation"))
    raw_input("press enter when done looking")
    del Globals.handles[:]
    return xyz2pc(good_xyzs, cloud_pc2.header.frame_id)

# prompt for a point cloud for a single object
def do_segmentation(obj_name):
    seg_svc = rospy.ServiceProxy("/interactive_segmentation", ProcessCloud)
    pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)
    pc_tf = ros_utils.transformPointCloud2(pc, exec_verb_traj.Globals.pr2.tf_listener, "base_footprint", pc.header.frame_id)
    print "select the %s" % (obj_name)
    pc_sel = seg_svc.call(ProcessCloudRequest(cloud_in = pc_tf)).cloud_out
    return pc_sel

# asks for all point clouds at once before execution
def get_all_clouds_pc2(num_objs):
    clouds = []
    for obj_num in xrange(num_objs):
        next_cloud = filter_pc2(do_segmentation("object%i" % obj_num))
        while not yes_or_no("Continue?"):
            next_cloud = filter_pc2(do_segmentation("object%i" % obj_num))
        clouds.append(next_cloud)
    return clouds

# execute for a single stage test (manually do the previous stage)
def do_single(demo_name, stage_num, prev_demo_index, verb_data_accessor, prev_and_cur_pc2):
    if stage_num == 0:
        do_stage(demo_name, stage_num, None, None, prev_and_cur_pc2[1], verb_data_accessor)
    else:
        prev_stage_num = stage_num - 1
        prev_demo_name = "%s%i" % (demo_base_name, prev_demo_index)
        prev_stage_info = verb_data_accessor.get_stage_info(prev_demo_name, prev_stage_num)

        call_and_print("rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller l_arm_controller")
        print colorize("do the stage involving the %s" % (prev_stage_info.item), color="red", bold=True)
        yes_or_no("type y to continue")
        call_and_print("rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller l_arm_controller")

        do_stage(demo_name, stage_num, prev_stage_info, prev_and_cur_pc2[0], prev_and_cur_pc2[1], verb_data_accessor)

# execute a single stage
def do_stage(demo_name, stage_num, prev_stage_info, prev_exp_pc2, cur_exp_pc2, verb_data_accessor):
    stage_info = verb_data_accessor.get_stage_info(demo_name, stage_num)
    make_req = get_trajectory_request(stage_info.verb, cur_exp_pc2)

    if stage_num == 0:
        grip_to_world_transform_func = None
    else:
        grip_to_world_transform_func = multi_item_make_verb_traj.make_grip_to_world_transform_tf("%s_gripper_tool_frame" %
                                                                                                 verb_data_accessor.get_stage_info(demo_name, stage_num).arms_used)

    make_resp = multi_item_make_verb_traj.make_traj_multi_stage(make_req, demo_name,
                                                                stage_num, prev_stage_info,
                                                                prev_exp_pc2, verb_data_accessor,
                                                                "tps_zrot")
    
    can_move_lower = (stage_num == 0)
    yn = yes_or_no("continue?")
    if yn:
        exec_req = ExecTrajectoryRequest()
        exec_req.traj = make_resp.traj
        exec_verb_traj.exec_traj(exec_req, traj_ik_func=ik_functions.do_traj_ik_graph_search,
                                 obj_pc=cur_exp_pc2, obj_name=stage_info.item, can_move_lower=can_move_lower)

# do a full experiment using the specified stages (stages are from different demos)
# demo base name is the verb and items without the index (e.g. the demo base name for demo pour-cup-bowl0 is pour-cup-bowl)
def do_multiple_varied_demos(demo_base_name, stages, verb_data_accessor, all_clouds_pc2):
    prev_stage_info, prev_exp_pc2 = None, None
    for (stage_num, (demo_num, cur_exp_pc2)) in enumerate(zip(stages, all_clouds_pc2)):
        demo_name = "%s%i" % (demo_base_name, demo_num)
        do_stage(demo_name, stage_num, prev_stage_info, prev_exp_pc2, cur_exp_pc2, verb_data_accessor)
        prev_stage_info = verb_data_accessor.get_stage_info(demo_name, stage_num)
        prev_exp_pc2 = cur_exp_pc2

# do a full experiment using all stages from the specified demo
def do_multiple_single_demo(demo_name, verb_data_accessor, all_clouds_pc2):
    prev_stage_info, prev_exp_pc2 = None, None
    for stage_num, cur_exp_pc2 in enumerate(all_clouds_pc2):
        do_stage(demo_name, stage_num, prev_stage_info, prev_exp_pc2, cur_exp_pc2, verb_data_accessor)
        prev_stage_info = verb_data_accessor.get_stage_info(demo_name, stage_num)
        prev_exp_pc2 = cur_exp_pc2

def do_globals_setup():
    multi_item_make_verb_traj.Globals.setup()
    exec_verb_traj.Globals.setup()
    Globals.setup()

# initialize the pr2 position
def move_pr2_to_start_pos(pr2):
    HEAD_ANGLE = 1.1
    pr2.rgrip.open()
    pr2.lgrip.open()
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')
    pr2.head.set_pan_tilt(0, HEAD_ANGLE)
    pr2.join_all()

def get_args():
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

def find_closest_demo(verb, exp_clouds_pc2):
    exp_clouds = [pc2xyzrgb(cloud)[0] for cloud in exp_clouds_pc2]
    closest_demo_name = scene_diff.get_closest_demo(verb, exp_clouds)
    return closest_demo_name

# determines the type of experiment (single or full) to run based on the args
# calls the corresponding function to do the actual experiment (do_single, do_multiple_single_demo, do_multiple_varied_demos)
def run_exp(args, verb_data_accessor):
    demo_base_name = args.demo
    print "using demo base", args.demo
    if args.verb is not None:
        exp_clouds_pc2 = get_all_clouds_pc2(verb_data_accessor.get_num_stages_for_verb(args.verb))
        closest_demo_name = find_closest_demo(args.verb, exp_clouds_pc2)
        print "Closest demo is %s" % (closest_demo_name)
        do_multiple_single_demo(closest_demo_name, verb_data_accessor, exp_clouds_pc2)
    if args.single is not None:
        params = [int(num) for num in args.single.split(',')]
        # make sure there are three values or the stage number is zero
        if len(params) == 3:
            prev_and_cur_pc2 = get_all_clouds_pc2(2)
            demo_index, stage_num, prev_demo_index = params
        elif len(params) == 2 and params[1] == 0:
            prev_and_cur_pc2 = [None, get_all_clouds_pc2(1)[0]]
            demo_index, stage_num = params
            prev_demo_index = -1
        else:
            raise ValueError("Invalid 'single' argument")
        do_single(demo_base_name, demo_index, stage_num, prev_demo_index, verb_data_accessor, prev_and_cur_pc2)
    elif args.stages is not None:
        if args.stages[0] == 'a':
            demo_name = "%s%s" % (demo_base_name, args.stages[1:])
            exp_clouds_pc2 = get_all_clouds_pc2(verb_data_accessor.get_num_stages(demo_name))
            closest_demo_name = find_closest_demo(verb_data_accessor.get_verb_from_demo_name(demo_name), exp_clouds_pc2)
            print "Closest demo is %s" % (closest_demo_name)
            do_multiple_single_demo(demo_name, verb_data_accessor, exp_clouds_pc2)
        else:
            stages = [int(stage) for stage in args.stages.split(",")]
            exp_clouds_pc2 = get_all_clouds_pc2(len(stages))
            do_multiple_varied_demos(demo_base_name, stages, verb_data_accessor, exp_clouds_pc2)

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed": 
        rospy.init_node("multi_item_exec_one_demo", disable_signals=True)
    do_globals_setup()
    args = get_args()
    pr2 = exec_verb_traj.Globals.pr2
    verb_data_accessor = multi_item_verbs.VerbDataAccessor()
    move_pr2_to_start_pos(pr2)

    assert args.demo is not None or args.verb is not None
    # use 'stages' to use all stages, use 'single' for just one stage
    assert (args.stages is not None and args.single is None) or (args.stages is None and args.single is not None)

    if args.demo is not None:
        run_exp(args, verb_data_accessor)
