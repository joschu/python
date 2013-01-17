from brett2.PR2 import PR2
import jds_utils.conversions as conv
import numpy as np
import geometry_msgs.msg as gm
import trajectory_msgs.msg as tm
import scipy.interpolate as si
import rospy
from time import time,sleep
import roslib
from jds_utils.math_utils import linspace2d

        
if rospy.get_name() == "/unnamed":
    rospy.init_node("test_base_traj",anonymous=True, disable_signals=True)

brett = PR2.create()


n_steps = 10
ts = np.linspace(0,1,n_steps)

xya_cur = np.r_[0,0,0]
xya_targ = np.r_[1,0,0]
xyas = linspace2d(xya_cur, xya_targ, n_steps)

pub = rospy.Publisher("base_traj_controller/command", tm.JointTrajectory)

jt = tm.JointTrajectory()
for i in xrange(10):
    
    jtp = tm.JointTrajectoryPoint()
    jtp.time_from_start = rospy.Duration(ts[i])
    jtp.positions = xyas[i]
    jt.points.append(jtp)
    
pub.publish(jt)

