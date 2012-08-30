import kinematics.reachability as kr
import openravepy
from jds_utils import conversions
import numpy as np

poses = []
poses.append(((0,.6,1), (0,0,0,1)))
poses.append(((0,.7,1), (0,0,0,1)))
poses.append(((0,.8,1), (0,0,0,1)))

env = openravepy.Environment()
env.Load("./tablescene.xml")
assert env is not None
robot = env.GetRobots()[0]
leftarm = robot.GetManipulator("leftarm")

db = kr.ReachabilityDatabase("read")
(x, y), cost =  kr.find_min_cost_base(db, robot, "leftarm", poses, plotting=True)
print cost
base_pose = np.eye(4)
base_pose[0,3] = x
base_pose[1,3] = y
robot.SetTransform(base_pose)

mats = [conversions.trans_rot_to_hmat(trans,rot) for (trans,rot) in poses]
for mat in mats:
    solns = leftarm.FindIKSolutions(mat,2+16)
    print "num solutions",len(solns)