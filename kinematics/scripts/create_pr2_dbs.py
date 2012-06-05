import kinematics.reachability as kr
import openravepy

env = openravepy.Environment()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]
larm = robot.GetManipulator("leftarm")
db = kr.ReachabilityDatabase("write")
kr.generate_db(db, larm, [.35, 1.1], [-.6, .75], [-.4, .4], .03, .03, .03, 1)
kr.create_mirror(db, "leftarm","rightarm")