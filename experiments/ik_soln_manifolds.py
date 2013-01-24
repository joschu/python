import openravepy as rave
import atexit
import jds_utils.math_utils as mu
env = rave.RaveGetEnvironment(1)
if env is None:
    env = rave.Environment()
    env.StopSimulation()
    atexit.register(rave.RaveDestroy)
    env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]
manip = robot.GetManipulator("leftarm")


poses = mu.linspace2d([1,0,0,0,.3,0,.7],[1,0,0,0,.5,0,.5],100)

xs, ys = [],[]
for (i,pose) in enumerate(poses):
    for soln in manip.FindIKSolutions(rave.matrixFromPose(pose), 18):
        xs.append(i)
        ys.append(soln[2])
        
import matplotlib.pyplot as plt
    
plt.plot(xs, ys, '.')