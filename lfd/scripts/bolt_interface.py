#!/usr/bin/env python
import rospy
from jds_utils.shell import call_and_print
from jds_utils.colorize import colorize
import rospy
import std_msgs.msg as stdm
import time
from brett2.PR2 import PR2

def callback(msg):
    s = msg.data
    print "message:",colorize(s, "blue", bold=True)
    action_arg = s.split()
    
    
    assert len(action_arg) == 2
    
    action, arg = action_arg

    if action == "relax":
        assert action_arg[0] == "relax"
        pr2 = PR2.create()
        pr2.rgrip.open()
        pr2.lgrip.open()
        pr2.rarm.goto_posture('side')
        pr2.larm.goto_posture('side')
    elif action == "tie":
        assert arg == "figure8_knot" or arg == "overhand_knot"
        call_and_print("execute_task.py --task=%s --count_steps"%arg)
    elif action == "teach":
        print colorize("which arms will be used?", "red", bold=True)
        arms_used = raw_input()
        call_and_print("teach_verb.py %s %s %s"%(arg, "object", arms_used))
    elif action == "execute":
        call_and_print("exec_one_demo_traj.py --verb=%s"%arg)
        

rospy.init_node("bolt_interface", disable_signals = True)
listener = rospy.Subscriber("/bolt_verbs", stdm.String, callback)
for i in xrange(3):
    try:
        time.sleep(100000)
    except KeyboardInterrupt:
        print "caught ctrl-c"
        