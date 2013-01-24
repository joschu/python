#!/usr/bin/env python

import rospy
import os
from jds_utils.colorize import colorize
import subprocess, os, sys, datetime
from glob import glob
from lfd import multi_item_verbs, exec_verb_traj
from jds_utils.yes_or_no import yes_or_no
import argparse
import time
import os.path as osp
import lfd
import make_verb_library2
from brett2 import PR2

def call_and_print(cmd,color='green'):
    print colorize(cmd, color, bold=True)
    if not args.dry_run: subprocess.check_call(cmd, shell=True)

# prompts for a special point
def ask_special_point():
    while True:
        special_pt_in = raw_input("Specify the special point relative to the gripper ((x y z) in m), or (n)one: ")

        if special_pt_in in ["n", "none"]:
            return None

        try:
            special_pt = [float(pt) for pt in special_pt_in.split(" ")]
        except ValueError:
            print "Invalid special point input"
            continue
        if len(special_pt) == 3:
            return special_pt
        else:
            print "Invalid special point input"

# calls the scripts to save a point cloud
def get_point_cloud(name_for_motion, item_name):
    call_and_print("get_point_cloud.py %s" % (name_for_motion))
    print colorize("now, select the %s object" % (item_name), color="red", bold=True)
    # two point clouds need to be stored, so give first point cloud a suffix of '-1'
    call_and_print("manually_segment_point_cloud.py %s.npz --objs=%s --do_filtering --plotting" % \
                   (name_for_motion, item_name))

# record a trajectory for a certain stage
def record_trajectory(dry_run, name_for_motion):
    raw_input("Press enter when ready to record")
    print colorize("now, human, demonstrate the next action for %s on the robot" % (name_for_motion), color="red", bold=True)
    call_and_print("rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller l_arm_controller")
    try:
        old_tmp_bags = glob("tmpname*.bag")
        if not dry_run: 
            for bag in old_tmp_bags: 
                os.remove(bag)
        call_and_print("rosbag record /tf /joint_states /joy -o tmpname")
    except KeyboardInterrupt:
        pass
    call_and_print("rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller l_arm_controller")

# renames a recently recorded bag file using name_for_motion
def rename_bag_file(name_for_motion, n_tries):
    success = False
    for i in xrange(n_tries):
        try:
            bagname = glob("tmpname*.bag")[0]
            os.rename(bagname, "%s.bag" % (name_for_motion))
            success = True
        except IndexError:
            time.sleep(.1)
    if not success:
        raise Exception("couldn't get bag file")

# gets a point cloud and records a trajectory for a certain stage
def record_demo_stage(name_for_motion, item_name, data_dir):
    os.chdir(data_dir + "/images")
    get_point_cloud(name_for_motion, item_name)
    while not yes_or_no("Ready to continue?"):
        get_point_cloud(name_for_motion, item_name)

    os.chdir(data_dir + "/bags")
    record_trajectory(args.dry_run, name_for_motion)
    n_tries = 20
    if not args.dry_run:
        rename_bag_file(name_for_motion, n_tries)

# initialize the position of the pr2
def move_pr2_to_start_pos(pr2):
    HEAD_ANGLE = 1.1
    pr2.rgrip.open()
    pr2.lgrip.open()
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')
    pr2.head.set_pan_tilt(0, HEAD_ANGLE)
    pr2.join_all()

# find the next unused demo name for a verb
def get_new_demo_name(verb, items):
    verb_data_accessor = multi_item_verbs.VerbDataAccessor()
    all_demo_info = verb_data_accessor.get_all_demo_info()
    for i in xrange(1000):
        demo_name = "%s-%s%i"%(verb, "-".join(items), i)
        if demo_name not in all_demo_info:
            return demo_name

# record all stages for a demo and return the stage names and corresponding special points
# stage names are <demo_name>-<stage_number>
def do_teach_verb(demo_name, items, arms_used, data_dir):
    stage_names_for_demo = []
    special_pts = []
    for i in range(len(items)):
        demo_name_with_stage = "%s-%s" % (demo_name, i)
        stage_names_for_demo.append(demo_name_with_stage)
        record_demo_stage(demo_name_with_stage, items[i], data_dir)

        # specify the special point for the item
        special_pt = ask_special_point()
        special_pts.append("None" if special_pt is None else special_pt)

    return stage_names_for_demo, special_pts

# creates the text for the yaml file for the demo
def get_new_demo_entry_text(demo_name, stage_names_for_demo, items, arms_used, special_pts):
            
    new_entry_text = """
# AUTOMATICALLY ADDED BY multi_item_teach_verb.py ON %(datestr)s
%(demo_name)s:
    stages: %(stages)s
    verb: %(verb)s
    items: %(items)s
    arms_used: %(arms_used)s
    special_pts : %(special_pts)s

    """ % dict(
            demo_name = demo_name,
            datestr = datetime.date.today(),
            stages = str(stage_names_for_demo),
            verb = verb,
            items = str(items),
            arms_used = str(arms_used),
            special_pts = str(special_pts),
            )

    return new_entry_text

# appends the entry text to the yaml file
def add_new_entry_to_yaml_file(data_dir, new_entry_text):
    demofile = open(data_dir + "/multi_item_verb_demos2.yaml", "a")
    demofile.write(new_entry_text)
    demofile.flush()
    os.fsync(demofile.fileno())
    demofile.close()
    call_and_print("make_verb_library2.py multi")

### START DEMONSTRATION ###

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("verb")
    parser.add_argument("items")
    parser.add_argument("arms_used")
    parser.add_argument("--dry_run", action="store_true")
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    if rospy.get_name() == "/unnamed": 
        rospy.init_node("multi_item_teach_verb", disable_signals=True)

    args = get_args()
    # arguments need to be: <verb> <items separated by ','> <arms_used separated by ',' and corresponding to items>
    verb = args.verb
    items_str = args.items
    items = items_str.split(",")
    arms_used = args.arms_used.split(",")
    data_dir = osp.join(osp.dirname(lfd.__file__), "data")

    pr2 = PR2.PR2()
    move_pr2_to_start_pos(pr2)

    demo_name = get_new_demo_name(verb, items)
    stage_names_for_demo, special_pts = do_teach_verb(demo_name, items, arms_used, data_dir)
    new_entry_text = get_new_demo_entry_text(demo_name, stage_names_for_demo, items, arms_used, special_pts)

    yn = yes_or_no("save demonstration?")
    # add the entry to the verbs yaml file
    if yn:
        add_new_entry_to_yaml_file(data_dir, new_entry_text)
    else:
        print "exiting without saving"
