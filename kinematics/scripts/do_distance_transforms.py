from kinematics import reachability
import numpy as np
import openravepy

env = openravepy.Environment()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]
rightarm = robot.GetManipulator("rightarm")
leftarm = robot.GetManipulator("leftarm")

db = reachability.ReachabilityDatabase("write")

reachability.create_distance_arrays(db)
