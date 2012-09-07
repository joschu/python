#!/usr/bin/env python

from jds_utils.colorize import colorize
import subprocess, os, sys, datetime
from glob import glob
from lfd import verbs
from jds_utils.yes_or_no import yes_or_no
import argparse
import time

def call_and_print(cmd,color='green'):
    print colorize(cmd, color, bold=True)
    if not args.dry_run: subprocess.check_call(cmd, shell=True)

parser = argparse.ArgumentParser()
parser.add_argument("verb")
parser.add_argument("target")
parser.add_argument("arms_used")
parser.add_argument("--dry_run", action="store_true")

args = parser.parse_args()
verb = args.verb
target = args.target
arms_used = args.arms_used

from lfd import verbs
all_demo_info = verbs.get_all_demo_info()
for i in xrange(1000):
    demo_name = "%s-%s%i"%(verb, target, i)
    if demo_name not in all_demo_info:
        break


    
os.chdir("/home/joschu/python/lfd/data/images")
call_and_print("get_point_cloud.py %s"%demo_name)
print colorize("now, select the target object", color="red", bold=True)
call_and_print("manually_segment_point_cloud.py %s.npz --objs=%s"%(demo_name, target))

print colorize("now, human, demonstrate the action on the robot", color="red", bold=True)
os.chdir("/home/joschu/python/lfd/data/bags")

call_and_print("rosrun pr2_controller_manager pr2_controller_manager stop r_arm_controller l_arm_controller")

try:
    old_tmp_bags = glob("tmpname*.bag")
    if not args.dry_run: 
        for bag in old_tmp_bags: 
            os.remove(bag)
    call_and_print("rosbag record /tf /joint_states /joy -o tmpname")
except KeyboardInterrupt:
    pass
n_tries = 20
if not args.dry_run:
    success = False
    for i in xrange(20):
        try:
            bagname = glob("tmpname*.bag")[0]
            os.rename(bagname, "%s.bag"%demo_name)
            success = True
        except IndexError:
            time.sleep(.1)
    if not success: raise Exception("couldn't get bag file")
call_and_print("rosrun pr2_controller_manager pr2_controller_manager start r_arm_controller l_arm_controller")

new_entry_text = """
# AUTOMATICALLY ADDED BY teach_verb.py ON %(datestr)s
%(demo_name)s:
  bag_file: bags/%(demo_name)s.bag
  seg_file: images/%(demo_name)s.seg.h5
  verb: %(verb)s
  args: [%(target)s]
  arms_used: %(arms_used)s
  
"""%dict(
       datestr = datetime.date.today(),
       demo_name = demo_name,
       verb = verb,
       target = target,
       arms_used = arms_used)

yn = yes_or_no("save demonstration?")
if yn:
    with open("/home/joschu/python/lfd/data/verb_demos2.yaml", "a") as demofile:
        if not args.dry_run: demofile.write(new_entry_text)
    call_and_print("make_verb_library2.py")
else:
    print "exiting without saving"