import kinematics.reachability as kr
import openravepy
from jds_utils import conversions
import numpy as np

lposes = []
lposes.append(((0,.6,1), (0,0,0,1)))
lposes.append(((0,.63,1), (0,0,0,1)))

rposes = lposes

env = openravepy.Environment()
env.Load("./tablescene.xml")
assert env is not None
robot = env.GetRobots()[0]
leftarm = robot.GetManipulator("leftarm")
rightarm = robot.GetManipulator("rightarm")

db = kr.ReachabilityDatabase("read")
(x, y), cost = kr.find_min_cost_base_bimanual(db, robot, lposes, rposes, True)
print "cost: ",cost
base_pose = np.eye(4)
base_pose[0,3] = x
base_pose[1,3] = y
robot.SetTransform(base_pose)

lmats = [conversions.trans_rot_to_hmat(trans,rot) for (trans,rot) in lposes]
rmats = [conversions.trans_rot_to_hmat(trans,rot) for (trans,rot) in rposes]
for (lmat,rmat) in zip(lmats,rmats):
    lsolns = leftarm.FindIKSolutions(lmat,2+16)
    rsolns = rightarm.FindIKSolutions(rmat,2+16)
    print "%i,%i"%(len(lsolns), len(rsolns))