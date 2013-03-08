import argparse
parser = argparse.ArgumentParser()
parser.add_argument("mode", choices=["openrave", "gazebo", "reality"])
args = parser.parse_args()

import trajoptpy
import openravepy
import numpy as np
import json
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku
import trajoptpy.make_kinbodies as mk

import functools as ft

import os
import os.path as osp
from glob import glob    

#########################
## Set up
#########################

if args.mode == "openrave":
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    import lfd
    env.Load(osp.join(osp.dirname(lfd.__file__), "data/table2.xml"))
    robot = env.GetRobots()[0]
    torso_joint = robot.GetJoint("torso_lift_joint")
    robot.SetDOFValues(torso_joint.GetLimits()[1], [torso_joint.GetDOFIndex()])
else:
    import rospy
    from brett2.PR2 import PR2
    rospy.init_node("follow_pose_traj",disable_signals=True)
    brett = PR2()
    env = brett.env
    robot = brett.robot
    if args.mode == "gazebo":
        brett.torso.go_up()
        rospy.sleep(1)
    brett.update_rave()
    
    if False:#args.mode == "reality":
        table_bounds = map(float, rospy.get_param("table_bounds").split())
        mk.create_box_from_bounds(env,table_bounds, name="table")       
    else:
        import lfd
        env.Load(osp.join(osp.dirname(lfd.__file__), "data/table2.xml"))
        
#######################



def adaptive_resample(x, tol, max_change=None, min_steps=3):
    """
    resample original signal it with a small number of waypoints so that the the sparsely sampled function, when linearly interpolated, 
    deviates from the original function by less than tol at every time
    
    input:
    x: 2D array in R^(t x k)  where t is the number of timesteps
    tol: tolerance. either a single scalar or a vector of length k
    max_change: max change in the sparsely sampled signal at each timestep
    min_steps: minimum number of timesteps in the new trajectory. (usually irrelevant)
    
    output:
    new_times, new_x
    
    assuming that the old signal has times 0,1,2,...,len(x)-1
    this gives the new times, and the new signal
    """
    x = np.asarray(x)
    assert x.ndim == 2
    
    if np.isscalar(tol): 
        tol = np.ones(x.shape[1])*tol
    else:
        tol = np.asarray(tol)
        assert tol.ndim == 1 and tol.shape[0] == x.shape[1]
    
    times = np.arange(x.shape[0])
    
    if max_change is None: 
        max_change = np.ones(x.shape[1]) * np.inf
    elif np.isscalar(max_change): 
        max_change = np.ones(x.shape[1]) * max_change
    else:
        max_change = np.asarray(max_change)
        assert max_change.ndim == 1 and max_change.shape[0] == x.shape[1]
        
    dl = mu.norms(x[1:] - x[:-1],1)
    l = np.cumsum(np.r_[0,dl])
    
    def bad_inds(x1, t1):
        ibad = np.flatnonzero( (np.abs(mu.interp2d(l, l1, x1) - x) > tol).any(axis=1) )
        jbad1 = np.flatnonzero((np.abs(x1[1:] - x1[:-1]) > max_change[None,:]).any(axis=1))
        if len(ibad) == 0 and len(jbad1) == 0: return []
        else:
            lbad = l[ibad]
            jbad = np.unique(np.searchsorted(l1, lbad)) - 1
            jbad = np.union1d(jbad, jbad1)
            return jbad
            
    
    l1 = np.linspace(0,l[-1],min_steps)
    for _ in xrange(20):
        x1 = mu.interp2d(l1, l, x)
        bi = bad_inds(x1, l1)
        if len(bi) == 0:
            return np.interp(l1, l, times), x1
        else:
            l1 = np.union1d(l1, (l1[bi] + l1[bi+1]) / 2 )
        
            
    raise Exception("couldn't subdivide enough. something funny is going on. check oyur input data")
        
        
def follow_traj_request(robot, manip_name, new_hmats, old_joints):
    n_steps = len(new_hmats)
    assert len(old_joints) == n_steps
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : manip_name,
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        },
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025]}
        }                
        ],
        "constraints" : [
        ],
        "init_info" : {
            "type":"given_traj",
            "data":[x.tolist() for x in old_joints]
        }
    }

    poses = [openravepy.poseFromMatrix(hmat) for hmat in new_hmats]
    for (i_step,pose) in enumerate(poses):
        request["costs"].append(
            {"type":"pose",
             "params":{
                "xyz":pose[4:7].tolist(),
                "wxyz":pose[0:4].tolist(),
                "link":"%s_gripper_tool_frame"%manip_name[0],
                "timestep":i_step
             }
            })


    
    return request        
        
###################################
###### Load demonstration files
###################################


IROS_DATA_DIR = os.getenv("IROS_DATA_DIR") 
def keyfunc(fname): return int(osp.basename(fname).split("_")[0][2:]) # sort files with names like pt1_larm.npy
lgrip_files, rgrip_files, larm_files, rarm_files = [sorted(glob(osp.join(IROS_DATA_DIR, "InterruptedSutureTrajectories/pt*%s.npy"%partname)), 
                                                           key = keyfunc)
                                                         for partname in ("lgrip", "rgrip", "larm", "rarm")]


######################################

from collections import namedtuple
TrajSegment = namedtuple("TrajSegment", "larm_traj rarm_traj lgrip_angle rgrip_angle") # class to describe trajectory segments



OPEN_ANGLE = .08
CLOSED_ANGLE = 0


def segment_trajectory(larm, rarm, lgrip, rgrip):
    
    thresh = .04 # open/close threshold
    
    n_steps = len(larm)
    assert len(rarm)==n_steps
    assert len(lgrip)==n_steps
    assert len(rgrip)==n_steps
    
    # indices BEFORE transition occurs
    l_openings = np.flatnonzero((lgrip[1:] >= thresh) & (lgrip[:-1] < thresh))
    r_openings = np.flatnonzero((rgrip[1:] >= thresh) & (rgrip[:-1] < thresh))
    l_closings = np.flatnonzero((lgrip[1:] < thresh) & (lgrip[:-1] >= thresh))
    r_closings = np.flatnonzero((rgrip[1:] < thresh) & (rgrip[:-1] >= thresh))

    before_transitions = np.r_[l_openings, r_openings, l_closings, r_closings]
    after_transitions = before_transitions+1
    seg_starts = np.unique1d(np.r_[0, after_transitions])
    seg_ends = np.unique1d(np.r_[before_transitions, n_steps])
    
    
    
    def binarize_gripper(angle):
        if angle > thresh: return OPEN_ANGLE
        else: return CLOSED_ANGLE
    
    traj_segments = []
    for (i_start, i_end) in zip(seg_starts, seg_ends):
        l_angle = binarize_gripper(lgrip[i_start])
        r_angle = binarize_gripper(rgrip[i_start])
        traj_segments.append(TrajSegment( larm[i_start:i_end], rarm[i_start:i_end], l_angle, r_angle))
    #import IPython
    #IPython.embed()
    return traj_segments

PARTNUM = 0
segments = segment_trajectory(
    np.load(larm_files[PARTNUM]),
    np.load(rarm_files[PARTNUM]),
    np.load(lgrip_files[PARTNUM]),
    np.load(rgrip_files[PARTNUM]))

print "trajectory broken into %i segments by gripper transitions"%len(segments)

robot.SetDOFValues(segments[0].larm_traj[0], robot.GetManipulator("leftarm").GetArmIndices())
robot.SetDOFValues(segments[0].rarm_traj[0], robot.GetManipulator("rightarm").GetArmIndices())

for (i,segment) in enumerate(segments):
    print "trajectory segment %i"%i
    
    full_traj = np.c_[segment.larm_traj, segment.rarm_traj]
    full_traj = mu.remove_duplicate_rows(full_traj)
    orig_times = np.arange(len(full_traj))
    ds_times, ds_traj =  adaptive_resample(full_traj, tol=.025, max_change=.1) # about 2.5 degrees, 10 degrees
    n_steps = len(ds_traj)

    
    
    ####################
    ### This part just gets the cartesian trajectory
    ####################
    
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("leftarm").GetArmIndices(), robot.GetManipulator("rightarm").GetArmIndices()])
    # let's get cartesian trajectory
    left_hmats = []
    right_hmats = []
    saver = openravepy.RobotStateSaver(robot)
    for row in ds_traj:
        robot.SetActiveDOFValues(row)
        left_hmats.append(robot.GetLink("l_gripper_tool_frame").GetTransform())
        right_hmats.append(robot.GetLink("r_gripper_tool_frame").GetTransform())
    saver.Restore()
    saver.Release()
    
    # now let's shift it a little
    for i in xrange(n_steps):
        left_hmats[i][0,3] -= .02
        right_hmats[i][0,3] -= .02
    
    ###################
    
    
    POSTURE_COEFF = 1
    trajoptpy.SetInteractive(False)
    #def nodecost(manip, joints, step):
        #robot = manip.GetRobot()
        #saver = openravepy.Robot.RobotStateSaver(robot)
        #robot.SetDOFValues(joints, manip.GetArmJoints(), False)
        #old_joints = ds_traj[step, :7] if manip.GetName() == "leftarm" else ds_traj[step, 7:]
        #joint_diff = old_joints - joints
        #for i in [2,4,6]: 
            #joint_diff[i] %= 2*np.pi
            #if joint_diff[i] > np.pi: joint_diff[i] -= np.pi
        #return 1000*robot.GetEnv().CheckCollision(robot) + POSTURE_COEFF * np.linalg.norm(joint_diff)
    #def left_ikfunc(hmat):
        #return ku.ik_for_link(hmat, robot.GetManipulator("leftarm"), "l_gripper_tool_frame", return_all_solns=True, filter_options = 1+2+16) # env collisions, no self / ee collisions
    #def unwrapped_squared_dist(x_nk,y_mk):
        #"pairwise squared distance between rows of matrices x and y, but mod 2pi on continuous joints"
        #diffs_nmk = np.abs(x_nk[:,None,:] - y_mk[None,:,:])
        #diffs_nmk[:,:,2] = diffs_nmk[:,:,2] % 2*np.pi
        #diffs_nmk[:,:,4] = diffs_nmk[:,:,4] % 2*np.pi
        #diffs_nmk[:,:,6] = diffs_nmk[:,:,6] % 2*np.pi
        #return (diffs_nmk**2).sum(axis=2)
    
    
    #left_paths, left_costs, timesteps = ku.traj_cart2joint(left_hmats, 
        #ikfunc = left_ikfunc,
        #nodecost=ft.partial(nodecost, robot.GetManipulator("leftarm")),
        #edgecost = unwrapped_squared_dist)
    
    #print "leftarm: IK succeeded on %s/%s timesteps. cost: %.2f"%(len(timesteps), n_steps, np.min(left_costs))
    #def unwind_arm_traj(x_nk):
        #out_nk = x_nk.copy()
        #for i in [2,4,6]:
            #out_nk[:,i] = np.unwrap(out_nk[:,i])
        #return out_nk    
    #best_left_path_before_interp = left_paths[np.argmin(left_costs)]
    #best_left_path_before_interp = unwind_arm_traj(best_left_path_before_interp)
    
    #if len(timesteps) < n_steps:
        #print "linearly interpolating the points with no soln"
        #best_left_path = mu.interp2d(np.arange(n_steps), timesteps, best_left_path_before_interp)
    #else: best_left_path = best_left_path_before_interp
    
    init_traj = ds_traj[:,:7].copy()
    init_traj[0] = robot.GetDOFValues(robot.GetManipulator("leftarm").GetArmIndices())
    req = follow_traj_request(robot, "leftarm", left_hmats, init_traj)
    s = json.dumps(req)
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    best_left_path = result.GetTraj()
    


    
    #def right_ikfunc(hmat):
        #return ku.ik_for_link(hmat, robot.GetManipulator("rightarm"), "r_gripper_tool_frame", return_all_solns=True, filter_options = 1+16) # env collisions + self collisions
    
    
    #right_paths, right_costs, timesteps = ku.traj_cart2joint(right_hmats, 
        #ikfunc = right_ikfunc,
        #nodecost=ft.partial(nodecost, robot.GetManipulator("rightarm")),
        #edgecost = unwrapped_squared_dist)
    
    #print "rightarm: IK succeeded on %s/%s timesteps. cost: %.2f"%(len(timesteps), len(right_hmats), np.min(right_costs))
    #best_right_path_before_interp = right_paths[np.argmin(right_costs)]
    #best_left_path_before_interp = unwind_arm_traj(best_right_path_before_interp)
    #if len(timesteps) < n_steps:
        #print "linearly interpolating the points with no soln"
        #best_right_path = mu.interp2d(np.arange(n_steps), timesteps, best_right_path_before_interp)
    #else: best_right_path = best_right_path_before_interp
    
    #im = np.zeros((len(left_hmats), 61), bool)
    #for (i_step,hmat) in enumerate(left_hmats):
        #solns = left_ikfunc(hmat)
        #if len(solns) > 0:
            #im[i_step,:][  (10*np.array(solns)[:,2]+3).astype('int') ] = 1
    #import matplotlib.pyplot as plt
    #plt.imshow(im)
    #plt.show()
    
    
    init_traj = ds_traj[:,7:14].copy()
    init_traj[0] = robot.GetDOFValues(robot.GetManipulator("rightarm").GetArmIndices())
    req = follow_traj_request(robot, "rightarm", right_hmats, init_traj)
    s = json.dumps(req)
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    best_right_path = result.GetTraj()
    
    
    left_diffs = np.abs(best_left_path[1:] - best_left_path[:-1])        
    right_diffs = np.abs(best_right_path[1:] - best_right_path[:-1])        
    print "max joint discontinuities in left arm:", left_diffs.max(), "per joint: ", left_diffs.max(axis=0)
    print "max joint discontinuities in right arm:", right_diffs.max(), "per joint: ", right_diffs.max(axis=0)    
    
    ##################################
    #### Now view/execute the trajectory
    ##################################
    
    if args.mode == "openrave":
        robot.SetActiveDOFs(np.r_[robot.GetManipulator("leftarm").GetArmIndices(), robot.GetManipulator("rightarm").GetArmIndices()])
        viewer = trajoptpy.GetViewer(env)
        joint_traj = np.c_[best_left_path, best_right_path]
        # *5 factor is due to an openrave oddity
        robot.SetDOFValues([segment.lgrip_angle*5, segment.rgrip_angle*5], [robot.GetJoint("l_gripper_l_finger_joint").GetDOFIndex(), robot.GetJoint("r_gripper_l_finger_joint").GetDOFIndex()], False)
        for (i,row) in enumerate(joint_traj):
            print "step",i
            robot.SetActiveDOFValues(row)
            lhandle = env.drawarrow(robot.GetLink("l_gripper_tool_frame").GetTransform()[:3,3], left_hmats[i][:3,3])
            rhandle = env.drawarrow(robot.GetLink("r_gripper_tool_frame").GetTransform()[:3,3], right_hmats[i][:3,3])
            viewer.Idle()
    else:
        from brett2 import trajectories
        #def follow_body_traj2(pr2, bodypart2traj, times=None, wait=True, base_frame = "/base_footprint"):
        bodypart2traj = {}
        brett.lgrip.set_angle(segment.lgrip_angle)
        brett.rgrip.set_angle(segment.rgrip_angle)
        brett.join_all()
        bodypart2traj["l_arm"] = segment.larm_traj
        bodypart2traj["r_arm"] = segment.rarm_traj
        trajectories.follow_body_traj2(brett, bodypart2traj)
        
