import openravepy
env = openravepy.Environment()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]
l0 = robot.GetLinks()[0]
l1 = robot.GetLink("base_footprint")
print hash(l0)
print hash(l1)
print l0 == l1