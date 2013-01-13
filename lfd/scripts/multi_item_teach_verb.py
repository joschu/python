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
from lfd import verbs
import make_verb_library2
from brett2 import PR2

class Globals:
    pr2 = None
    isinstance(pr2, PR2.PR2)
    
    def __init__(self): raise

    @staticmethod
    def setup():
        Globals.pr2 = PR2.PR2.create()

def call_and_print(cmd,color='green'):
    print colorize(cmd, color, bold=True)
    if not args.dry_run: subprocess.check_call(cmd, shell=True)

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

def record_demonstration_for_motion(name_for_motion, item_name, data_dir):
    # get the point cloud for the first item
    os.chdir(data_dir + "/images")
    call_and_print("get_point_cloud.py %s" % (name_for_motion))
    print colorize("now, select the %s object" % (item_name), color="red", bold=True)
    # two point clouds need to be stored, so give first point cloud a suffix of '-1'
    call_and_print("manually_segment_point_cloud.py %s.npz --objs=%s" % (name_for_motion, item_name))

    yes_or_no("Ready to continue?")

    # do the demonstration for the first motion
    print colorize("now, human, demonstrate the next action for %s on the robot" % (name_for_motion), color="red", bold=True)
    os.chdir(data_dir + "/bags")

    call_and_print("rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller l_arm_controller")

    # record the demonstration for the first motion
    try:
        old_tmp_bags = glob("tmpname*.bag")
        if not args.dry_run: 
            for bag in old_tmp_bags: 
                os.remove(bag)
        call_and_print("rosbag record /tf /joint_states /joy -o tmpname")
    except KeyboardInterrupt:
        pass

    # rename bag file that was just created
    n_tries = 20
    if not args.dry_run:
        success = False
        for i in xrange(n_tries):
            try:
                bagname = glob("tmpname*.bag")[0]
                os.rename(bagname, "%s.bag" % (name_for_motion))
                success = True
            except IndexError:
                time.sleep(.1)
        if not success: raise Exception("couldn't get bag file")
    call_and_print("rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller l_arm_controller")

def move_pr2_to_start_pos(pr2):
    HEAD_ANGLE = 1.1
    pr2.rgrip.open()
    pr2.lgrip.open()
    pr2.rarm.goto_posture('side')
    pr2.larm.goto_posture('side')
    pr2.head.set_pan_tilt(0, HEAD_ANGLE)
    pr2.join_all()

def get_new_demo_name(verb, items):
    verb_data_accessor = multi_item_verbs.VerbDataAccessor()
    all_demo_info = verb_data_accessor.get_all_demo_info()
    for i in xrange(1000):
        demo_name = "%s-%s%i"%(verb, "-".join(items), i)
        if demo_name not in all_demo_info:
            return demo_name

def get_new_demo_entry_text(demo_name, items, arms_used, data_dir):
    demo_names_with_stage = []
    special_pts = []
    for i in range(len(items)):
        demo_name_with_stage = "%s-%s" % (demo_name, i)
        demo_names_with_stage.append(demo_name_with_stage)
        record_demonstration_for_motion(demo_name_with_stage, items[i], data_dir)

        # specify the special point for the item
        special_pt = ask_special_point()
        special_pts.append("None" if special_pt is None else special_pt)
            
    new_entry_text = """
# AUTOMATICALLY ADDED BY two_item_teach_verb.py ON %(datestr)s
%(demo_name)s:
    stages: %(stages)s
    verb: %(verb)s
    args: %(items)s
    arms_used: %(arms_used)s
    special_pts : %(special_pts)s

    """ % dict(
            demo_name = demo_name,
            datestr = datetime.date.today(),
            stages = str(demo_names_with_stage),
            verb = verb,
            items = str(items),
            arms_used = str(arms_used),
            special_pts = str(special_pts),
            )

    return new_entry_text

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

    Globals.setup()
    move_pr2_to_start_pos(Globals.pr2)

    demo_name = get_new_demo_name(verb, items)

    new_entry_text = get_new_demo_entry_text(demo_name, items, arms_used, data_dir)

    yn = yes_or_no("save demonstration?")
    # add the entry to the verbs yaml file
    if yn:
        add_new_entry_to_yaml_file(data_dir, new_entry_text)
    else:
        print "exiting without saving"
