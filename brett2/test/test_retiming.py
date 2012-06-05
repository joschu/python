import numpy as np
import pr2
from numpy import pi

brett = pr2.PR2()

#print "executing raw trajectory"
#brett.larm.follow_timed_joint_traj(shake["l_pos"],shake["l_vel"], shake["times"])

print "executing smoothed trajectory"

larm = brett.larm
postures = larm.L_POSTURES
traj = np.array([
    postures["untucked"],
    postures["tucked"],
    postures["up"],
    postures["side"]])
brett.larm.follow_joint_traj(traj)