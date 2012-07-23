import rospy
import numpy as np
import sensor_msgs.msg as sm
import roslib
roslib.load_manifest("pr2_controllers_msgs"); 
import pr2_controllers_msgs.msg as pcm
import matplotlib.pyplot as plt

JOINT_NAME = "l_gripper_joint"
CMD_TOPIC = "/l_gripper_controller/command"
rospy.init_node("check_command_vs_actual", disable_signals = True)


cmds = []
acts = []
def joint_cb(msg):
    acts.append((rospy.Time.now(),msg.position[msg.name.index(JOINT_NAME)]))
def cmd_cb(msg):
    cmds.append((rospy.Time.now(),msg.position))

cmd_sub = rospy.Subscriber(CMD_TOPIC, pcm.Pr2GripperCommand, cmd_cb)
joint_sub = rospy.Subscriber("/joint_states", sm.JointState, joint_cb)


print "sleeping until ctrl-c"
try:
    rospy.sleep(100)
except KeyboardInterrupt:
    pass


cmd_times, cmd_vals = zip(*cmds)
act_times, act_vals = zip(*acts)



plt.plot([t.to_sec() for t in cmd_times], cmd_vals,'b-x')
plt.plot([t.to_sec() for t in act_times], act_vals,'g-')
plt.legend(["cmd","act"])
plt.savefig("cmd_vs_actual.png")
plt.show()
