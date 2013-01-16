from openravepy import *
import numpy, time
import jds_utils.conversions as conv
import traj_opt, traj_ik_graph_search
from progress.bar import Bar

env=Environment()
env.Load('robots/pr2-beta-static.zae')

robot=env.GetRobots()[0]
manip = robot.SetActiveManipulator('leftarm')
lower,upper = robot.GetDOFLimits(manip.GetArmIndices()) # get the limits of just the arm indices
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()


n_iter = 1000
rand_joints = []
for i in range(n_iter):
    rand_joints.append(lower+numpy.random.rand(len(lower))*(upper-lower))
rand_hmats = [conv.trans_rot_to_hmat(*traj_opt.joints2xyzquat(manip, 'l_gripper_tool_frame', j)) for j in rand_joints]


start_time = time.time()
solns = []
for hmat in Bar().iter(rand_hmats):
    s = numpy.array(traj_ik_graph_search.ik_for_link(hmat, manip, 'l_gripper_tool_frame', return_all_solns=True))
    if s.size == 0: continue
    solns.append(s)
end_time = time.time()
print 'ik time: ', end_time-start_time, ' | ', (end_time-start_time)/n_iter, 'each'

start_time = time.time()
for i in Bar().iter(range(len(solns)-1)):
    a, b, c = traj_ik_graph_search.build_graph_part(None, solns[i], solns[i+1])
end_time = time.time()
print 'pairwise dist time: ', end_time-start_time, ' | ', (end_time-start_time)/n_iter, 'each'
