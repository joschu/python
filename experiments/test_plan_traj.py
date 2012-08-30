from brett2 import PR2
from bulletsim_msgs.srv import *
import rospy
import numpy as np


PLANNER = None
def goto_joint_positions_planned(arm, jpos):

    global PLANNER
    if PLANNER is None: PLANNER = rospy.ServiceProxy("/plan_traj", PlanTraj)

    assert isinstance(arm, PR2.Arm)
    req = PlanTrajRequest()
    req.start_joints = arm.get_joint_positions()
    req.end_joints = jpos
    req.side = arm.lr
    
    resp = PLANNER.call(req)
    assert isinstance(resp, PlanTrajResponse)
    traj = np.asarray(resp.trajectory).reshape(-1,7)
    arm.follow_joint_trajectory(traj)
    
    

rospy.init_node("test_plan_svc",disable_signals=True)
pr2 = PR2.PR2()
pr2.larm.goto_posture('side')
pr2.join_all()
pr2.larm.goto_joint_positions_planned(pr2.larm.L_POSTURES['up'])