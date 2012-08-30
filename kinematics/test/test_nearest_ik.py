import kinematics.reachability as kr
import openravepy
import jds_utils.conversions
import numpy as np
from time import time

env = openravepy.Environment()
env.Load("./tablescene.xml")
assert env is not None
robot = env.GetRobots()[0]
leftarm = robot.GetManipulator("leftarm")
db = kr.ReachabilityDatabase("read")

results = []
n_tries = 1000

t_start = time()
for i in xrange(n_tries):
    success = kr.get_nearest_ik(db, leftarm, 100*np.random.randn(3), np.r_[0,0,0,1]) is not None
    results.append(success)
    
print "success fraction:", np.mean(results)
print "time per call", (time() - t_start)/n_tries