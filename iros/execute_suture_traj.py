import argparse
parser = argparse.ArgumentParser()
parser.add_argument("mode", choices=["openrave", "gazebo", "reality"])
parser.add_argument("--cloud_topic", default="/camera/depth_registered/points")
parser.add_argument("task")
parser.add_argument("part_index",type=int)
parser.add_argument("--segment_index", type=int, default=0)
parser.add_argument("--interactive",action="store_true")
args = parser.parse_args()

import numpy as np
import trajoptpy
import lfd
import openravepy
import json
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku
import trajoptpy.make_kinbodies as mk
import brett2.ros_utils as ru
from jds_utils.colorize import colorize
from jds_utils import conversions
from brett2 import mytf
import yaml
import cv2

import functools as ft
import simple_clicker as sc
import os
import os.path as osp
from glob import glob
import subprocess, sys
import sensor_msgs.msg as sm

from time import sleep

window_name = "Find Keypoints"
cv2.namedWindow(window_name)

#########################
###### Set up
#########################

IROS_DATA_DIR = os.getenv("IROS_DATA_DIR")

import iros
IROS_DIR = osp.dirname(iros.__file__)
task_file = osp.join(IROS_DIR, "suture_demos2.yaml")

with open(osp.join(IROS_DATA_DIR,task_file),"r") as fh:
    task_info = yaml.load(fh)

jtf = osp.join(IROS_DATA_DIR, args.task, 'joint_trajectories', 'pt' + str(args.part_index)) 
kpf = osp.join(IROS_DATA_DIR, args.task, 'keypoints', 'pt' + str(args.part_index))
pcf = osp.join(IROS_DATA_DIR, args.task, 'point_clouds', 'pt' + str(args.part_index))

if args.mode == "openrave":
    env = openravepy.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    env.Load(osp.join(osp.dirname(lfd.__file__), "data/table2.xml"))
    robot = env.GetRobots()[0]
    torso_joint = robot.GetJoint("torso_lift_joint")
    robot.SetDOFValues(torso_joint.GetLimits()[1], [torso_joint.GetDOFIndex()])
    
else:
    import rospy
    from brett2.PR2 import PR2
    if rospy.get_name() == "/unnamed": rospy.init_node("follow_pose_traj", disable_signals=True)
    rviz = ru.RvizWrapper()
    
brett = PR2()
env = brett.env
robot = brett.robot
if args.mode == "gazebo":
    brett.torso.go_up()
    rospy.sleep(1)
brett.head.set_pan_tilt(0,1)
brett.join_all()
brett.update_rave()

#needle_tip = mk.create_dummy_body(env, name="needle_tip")
needle_tip = mk.create_spheres(env, [(0,0,0)], radii=.02, name = "needle_tip")

demo_env=env.CloneSelf(1)
demo_env.StopSimulation()
demo_robot = demo_env.GetRobot("pr2")
demo_needle_tip = demo_env.GetKinBody("needle_tip")

trajoptpy.SetInteractive(args.interactive)

if False:#args.mode == "reality":
    table_bounds = map(float, rospy.get_param("table_bounds").split())
    mk.create_box_from_bounds(env,table_bounds, name="table")       
else:
    import lfd
    env.Load(osp.join(osp.dirname(lfd.__file__), "data/table2.xml"))

#######################

from collections import namedtuple
TrajSegment = namedtuple("TrajSegment", "larm_traj rarm_traj lgrip_angle rgrip_angle") # class to describe trajectory segments

PARTNUM = args.part_index
SEGNUM = len(task_info[args.task][args.part_index]["segments"])
print 'number of segments', SEGNUM
OPEN_ANGLE = .08
CLOSED_ANGLE = 0


def call_and_print(cmd,color='green'):
    print colorize(cmd, color, bold=True)
    subprocess.check_call(cmd, shell=True)


def transform_hmats(f, hmats):
    hmats = np.array(hmats)
    oldpts_md = hmats[:,:3,3]
    oldrots_mad = hmats[:,:3,:3]
    newpts_mg, newrots_mgd = f.transform_frames(oldpts_md, oldrots_mad)    
    tf_hmats = hmats.copy()
    tf_hmats[:,:3,:3] = newrots_mgd
    tf_hmats[:,:3,3] = newpts_mg
    return tf_hmats

def translation_matrix(xyz):
    out = np.eye(4)
    out[:3,3] = xyz
    return out

def adaptive_resample(x, tol, max_change=None, min_steps=3):
    """
    resample original signal with a small number of waypoints so that the the sparsely sampled function, 
    when linearly interpolated, deviates from the original function by less than tol at every time

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


    raise Exception("couldn't subdivide enough. something funny is going on. check your input data")


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
    seg_starts = np.unique(np.r_[0, after_transitions])
    seg_ends = np.unique(np.r_[before_transitions, n_steps])


    def binarize_gripper(angle):
        if angle > thresh: return OPEN_ANGLE
        else: return CLOSED_ANGLE

    traj_segments = []
    for (i_start, i_end) in zip(seg_starts, seg_ends):
        l_angle = binarize_gripper(lgrip[i_start])
        r_angle = binarize_gripper(rgrip[i_start])
        traj_segments.append(TrajSegment( larm[i_start:i_end+1], rarm[i_start:i_end+1], l_angle, r_angle ))

    return traj_segments

#def get_kp_cloud(listener):
    #print "waiting for messages on topic %s"%args.cloud_topic
    #msg = rospy.wait_for_message(args.cloud_topic, sm.PointCloud2)
    #print "got msg!"
    #xyz, rgb = ru.pc2xyzrgb(msg)

    #if (xyz.shape[0] == 1 or xyz.shape[1] == 1): raise Exception("needs to be an organized point cloud")	

    #xyz_tf = ru.transform_points(xyz, listener, "base_footprint", "/camera_rgb_optical_frame")        
    #xyz_tf[np.isnan(xyz_tf)] = -2
    #rgb_plot = rgb.copy()
    
    #return xyz_tf, rgb_plot

def get_kp_clouds(listener, num_clouds):
    xyz_tfs = []
    rgb_plots = []
    #num_clouds = 30
       
    class MessageGetter:
        
        def __init__(self, n_msgs):
            self.n_msgs = n_msgs
            self.msgs = []
            self.done = False
        def callback(self,msg):
            if self.done:
                return
            elif len(self.msgs) < self.n_msgs:
                self.msgs.append(msg)
            else:
                self.done = True
                
    def get_msgs(n_msgs, topic, msgtype):
        print "waiting for %i messages on topic %s"%(n_msgs, topic)
        mg = MessageGetter(n_msgs)
        sub = rospy.Subscriber(topic, msgtype, mg.callback)
        while not mg.done:
            rospy.sleep(.05)
        sub.unregister()
        return mg.msgs
              
    msgs = get_msgs(num_clouds, args.cloud_topic, sm.PointCloud2)
    
    if num_clouds == 1:
        msg = msgs[0]
        xyz, rgb = ru.pc2xyzrgb(msg)
    
        if (xyz.shape[0] == 1 or xyz.shape[1] == 1): raise Exception("needs to be an organized point cloud")	
    
        xyz_tf = ru.transform_points(xyz, listener, "base_footprint", "/camera_rgb_optical_frame")        
        rgb_plot = rgb.copy() 
        
        return xyz_tf, rgb_plot

    else:    
        for i in range(num_clouds):
            msg = msgs[i]            
            xyz, rgb = ru.pc2xyzrgb(msg)
           
            if (xyz.shape[0] == 1 or xyz.shape[1] == 1): raise Exception("needs to be an organized point cloud")
    
            xyz_tf = ru.transform_points(xyz, listener, "base_footprint", "/camera_rgb_optical_frame")        
        
            xyz_tfs.append(xyz_tf)
            rgb_plots.append(rgb)
    
        return xyz_tfs, rgb_plots

#def GetLinkMaybeAttached(robot,ee_link):
    #link = robot.GetLink(ee_link)
    #if link is not None: return link
    #grabbed_bodies = robot.GetGrabbed()
    #for grabbed_body in grabbed_bodies:
        #link = grabbed_body.GetLink(ee_link)
        #if link: return link
    #return None

def plan_follow_traj(robot, manip_name, ee_link, new_hmats, old_traj, other_manip_name = None, other_manip_traj = None):

    n_steps = len(new_hmats)
    assert old_traj.shape[0] == n_steps
    assert old_traj.shape[1] == 7
    
    arm_inds  = robot.GetManipulator(manip_name).GetArmIndices()

    ee_linkname = ee_link.GetName()
    
    init_traj = old_traj.copy()
    init_traj[0] = robot.GetDOFValues(arm_inds)

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
            "params" : {"coeffs" : [10],"dist_pen" : [0.005]}
        }                
        ],
        "constraints" : [
        ],
        "init_info" : {
            "type":"given_traj",
            "data":[x.tolist() for x in init_traj]
        }
    }
    if other_manip_name is not None:
        request["scene_states"] = []
        other_dof_inds = robot.GetManipulator(other_manip_name).GetArmIndices()
        
    poses = [openravepy.poseFromMatrix(hmat) for hmat in new_hmats]
    for (i_step,pose) in enumerate(poses):
        request["costs"].append(
            {"type":"pose",
             "params":{
                "xyz":pose[4:7].tolist(),
                "wxyz":pose[0:4].tolist(),
                "link":ee_linkname,
                "timestep":i_step,
                "pos_coeffs":[15,15,15],
                "rot_coeff":[0,0,0]
             }
            })
        if other_manip_name is not None:
            request["scene_states"].append(
                {"timestep": i_step, "obj_states": [{"name": "pr2", "dof_vals":other_manip_traj[i_step].tolist(), "dof_inds":other_dof_inds.tolist()}] })


    s = json.dumps(request)
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    traj = result.GetTraj()    
        
    saver = openravepy.RobotStateSaver(robot)
    pos_errs = []
    for i_step in xrange(1,n_steps):
        row = traj[i_step]
        robot.SetDOFValues(row, arm_inds)
        tf = ee_link.GetTransform()
        pos = tf[:3,3]
        pos_err = np.linalg.norm(poses[i_step][4:7] - pos)
        pos_errs.append(pos_err)
    pos_errs = np.array(pos_errs)
        
    print "planned trajectory for %s. max position error: %.3f. all position errors: %s"%(manip_name, pos_errs.max(), pos_errs)
            
    return traj         




#######################################
###### Load demo from np files
#######################################

demo_keypts = np.load(osp.join(IROS_DATA_DIR, kpf + "/keypoints.npy"))

def keyfunc(fname): 
    return int(osp.basename(fname).split("_")[0][3:]) # sort files with names like seg0_larm.npy

lgrip_files, rgrip_files, larm_files, rarm_files = [sorted(glob(jtf + "/seg*%s.npy"%partname), 
                                                           key = keyfunc)
                                                    for partname in ("lgrip", "rgrip", "larm", "rarm")]

mini_segments = []

for s in range(SEGNUM):
    mini_segments.append( segment_trajectory( np.load(larm_files[s]),
                                              np.load(rarm_files[s]),
                                              np.load(lgrip_files[s]),
                                              np.load(rgrip_files[s])))

    print "trajectory segment", str(s), "broken into %i mini-segment(s) by gripper transitions"%len(mini_segments[s])

#raw_input("press enter to execute")
#######################################

### iterate through each 'look' segment; for each segment:
### click on key points
### compare to key points from demonstration, do registration, find warping function
###     iterate through each mini-segment
###     downsample
###     find cartestian coordinates for trajectory
###     transform cartestian trajectory according to warping function
###     do ik to find new joint space trajectory (left arm first, then right arm)
###     execute new trajectory

listener = ru.get_tf_listener()
handles = []
exec_keypts = {}   

if args.segment_index > 0: #HACK so we can start in the middle
    start_segment_info = task_info[args.task][args.part_index]["segments"][args.segment_index]
    if "left_end_effector" in start_segment_info:
        needle_tip.SetTransform(robot.GetManipulator("leftarm").GetTransform())
        robot.Grab(needle_tip)
        demo_needle_tip.SetTransform(demo_robot.GetManipulator("leftarm").GetTransform())
        demo_robot.Grab(demo_needle_tip)

for s in range(args.segment_index, SEGNUM):
    
    segment_info = task_info[args.task][args.part_index]["segments"][s]
    keypt_names = segment_info["keypts_to_look_for"]
    num_kps = len(keypt_names)
    
    print colorize("trajectory segment %i"%s, 'blue', bold=True, highlight=True)    

    # keep track of keypts seen during each segment
    exec_keypts[s] = {}
    exec_keypts[s]["names"] = []
    exec_keypts[s]["locations"] = []
    
    # this is the frame whose trajectory we'll adapt to the new situation
    # in some segments it's the needle tip
    left_ee_linkname = segment_info.get("left_end_effector", "l_gripper_tool_frame")
    right_ee_linkname = segment_info.get("right_end_effector", "r_gripper_tool_frame")
    
    
    if left_ee_linkname == "needle_tip":
        left_ee_link = needle_tip.GetLinks()[0]
        demo_left_ee_link = demo_needle_tip
    else:
        left_ee_link = robot.GetLink(left_ee_linkname)
        demo_left_ee_link = demo_robot.GetLink(left_ee_linkname)
    if right_ee_linkname == "needle_tip":
        right_ee_link = needle_tip.GetLinks()[0]
        demo_right_ee_link = demo_needle_tip
    else:
        right_ee_link = robot.GetLink(right_ee_linkname)
        demo_right_ee_link = demo_robot.GetLink(right_ee_linkname)
    
    brett.update_rave()
    for k in range(num_kps):
        print colorize("Key point from demo is: " + keypt_names[k] + ". Looking for this key point now...", 'green', bold=True)                       
        rospy.sleep(0.5)
        
        if keypt_names[k] == 'none':
            kp_loc = ((0, 0, 0))
            print colorize("No keypoint for this segment", 'green', bold=True)
        
        elif args.cloud_topic == 'test':
            xyz_tf = np.load(pcf + '/seg%s_' + keypt_names[k] + '_xyz_tf.npy'%s)
            rgb_plot = np.load(pcf + '/seg%s_' + keypt_names[k] + '_rgb_pl.npy'%s)  

        elif keypt_names[k] in ['left_hole', 'right_hole', 'stand', 'top_cut', 'mid_cut', 'bot_cut']:
            xyz_tf, rgb_plot = get_kp_clouds(listener, 1)                     
            kp_loc = sc.find_kp(keypt_names[k], xyz_tf, rgb_plot, window_name)            
           
        elif keypt_names[k] == 'needle_end':
            while True:
                xyz_tfs, rgb_plots = get_kp_clouds(listener, 30)        
                kp_loc, pts = sc.find_needle_end(xyz_tfs, rgb_plots, window_name)
                if pts > 0: break
                else: print colorize("Couldn't find the needle end! Trying again","red", True, True)                       
            
        elif keypt_names[k] == 'needle_tip':
            while True:
                xyz_tfs, rgb_plots = get_kp_clouds(listener, 30)
                kp_loc = sc.find_needle_tip(xyz_tfs, rgb_plots, window_name, 'execute')
                if kp_loc[2] > 0.8: break                
                else: print colorize("Didn't find a high enough point! Trying again","red", True, True)                    
                
        elif keypt_names[k] == 'tip_transform': # this is segment where robot looks for tip
            demo_needle_tip_loc = np.load(osp.join(IROS_DATA_DIR, kpf, "seg%s_needle_world_loc.npy"%s))

            while True:
                xyz_tfs, rgb_plots = get_kp_clouds(listener, 30)
                exec_needle_tip_loc = sc.find_needle_tip(xyz_tfs, rgb_plots, window_name, 'execute')
                if exec_needle_tip_loc[2] > 1: break                
                else: print colorize("Didn't find a high enough point! Trying again","red", True, True)
            
            kp_loc = ((0, 0, 0))
                                
            def grab_needle_tip(lr):
                for demo in [False, True]:
                    if demo: 
                        tip_loc = demo_needle_tip_loc
                        grabbing_robot = demo_robot
                        grabbed_needle_tip = demo_needle_tip
                    else: 
                        tip_loc = exec_needle_tip_loc
                        grabbing_robot = robot
                        grabbed_needle_tip = needle_tip
                    grabbed_needle_tip.SetTransform(translation_matrix(tip_loc))
                    grabbing_robot.Grab(grabbed_needle_tip, grabbing_robot.GetLink("%s_gripper_tool_frame"%lr))                
            
            if "extra_info" in segment_info:
                if "left_grab" in segment_info["extra_info"]: 
                    grab_needle_tip('l')
                elif "right_grab" in segment_info["extra_info"]:
                    grab_needle_tip('r') 
                    
        else:
            print "Invalid keypoint name! Aborting..."
            sys.exit(1)
            
        exec_keypts[s]["names"].append(keypt_names[k])
        exec_keypts[s]["locations"].append(kp_loc)                
                
    demopoints_m3 = np.array(demo_keypts[s])
    newpoints_m3 = np.array(exec_keypts[s]["locations"])
    
    if args.mode in ["gazebo", "reality"]:
        handles = []
        pose_array = conversions.array_to_pose_array(demopoints_m3, 'base_footprint')    
        handles.append(rviz.draw_curve(pose_array, rgba = (1,0,0,1),width=.02,type=ru.Marker.CUBE_LIST))
        pose_array = conversions.array_to_pose_array(newpoints_m3, 'base_footprint')    
        handles.append(rviz.draw_curve(pose_array, rgba = (0,0,1,1),width=.02,type=ru.Marker.CUBE_LIST))

    from lfd import registration
    f = registration.ThinPlateSpline()
    #f.fit(demopoints_m3, newpoints_m3, 10,10)
    f.fit(demopoints_m3, newpoints_m3, bend_coef=10,rot_coef=.01)
    np.set_printoptions(precision=3)
    print "nonlinear part", f.w_ng
    print "affine part", f.lin_ag
    print "translation part", f.trans_g
    print "residual", f.transform_points(demopoints_m3) - newpoints_m3



    for (i, mini_segment) in enumerate(mini_segments[s]):

        brett.update_rave()
        
        print colorize("mini-segment %i"%i, 'red', bold=True, highlight=True)

        full_traj = np.c_[mini_segment.larm_traj, mini_segment.rarm_traj]
        full_traj = mu.remove_duplicate_rows(full_traj)
        orig_times = np.arange(len(full_traj))

        ### downsample the trajectory
        ds_times, ds_traj =  adaptive_resample(full_traj, tol=.01, max_change=.1) # about 2.5 degrees, 10 degrees
        n_steps = len(ds_traj)

        ################################################
        ### This part gets the cartesian trajectory
        ################################################

        robot.SetActiveDOFs(np.r_[robot.GetManipulator("leftarm").GetArmIndices(), robot.GetManipulator("rightarm").GetArmIndices()])
        demo_robot.SetActiveDOFs(np.r_[robot.GetManipulator("leftarm").GetArmIndices(), robot.GetManipulator("rightarm").GetArmIndices()])        
        
        demo_robot.SetDOFValues(robot.GetDOFValues())
        
        # let's get cartesian trajectory
        left_hmats = []
        right_hmats = []

            
        for row in ds_traj:
            demo_robot.SetActiveDOFValues(row)
            left_hmats.append(demo_left_ee_link.GetTransform())
            right_hmats.append(demo_right_ee_link.GetTransform())


        left_hmats_old = left_hmats
        left_hmats = transform_hmats(f, left_hmats)
        right_hmats_old = right_hmats
        right_hmats = transform_hmats(f, right_hmats)


        if args.mode in ["gazebo", "reality"]:
            pose_array = conversions.array_to_pose_array(np.array(left_hmats_old)[:,:3,3], 'base_footprint')    
            handles.append(rviz.draw_curve(pose_array, rgba = (1,0,0,1),width=.005,type=ru.Marker.LINE_STRIP))
            pose_array = conversions.array_to_pose_array(np.array(right_hmats_old)[:,:3,3], 'base_footprint')    
            handles.append(rviz.draw_curve(pose_array, rgba = (1,0,0,1),width=.005,type=ru.Marker.LINE_STRIP))

            pose_array = conversions.array_to_pose_array(np.array(left_hmats)[:,:3,3], 'base_footprint')    
            handles.append(rviz.draw_curve(pose_array, rgba = (0,0,1,1),width=.005,type=ru.Marker.LINE_STRIP))
            pose_array = conversions.array_to_pose_array(np.array(right_hmats)[:,:3,3], 'base_footprint')    
            handles.append(rviz.draw_curve(pose_array, rgba = (0,0,1,1),width=.005,type=ru.Marker.LINE_STRIP))


        ################################################
        
        leftarm_inds = robot.GetManipulator("leftarm").GetArmIndices()
        rightarm_inds = robot.GetManipulator("rightarm").GetArmIndices()
        
        def remove_winding(arm_traj, current_arm_vals):   
            arm_traj = arm_traj.copy()
            for i in [2,4,6]:
                dof_jump = current_arm_vals[i] - arm_traj[0,i]
                winds = np.round(dof_jump / (2*np.pi))
                if winds > 0: print "unwound joint %i by 2pi*%i"%(i,winds)
                arm_traj[:,i] += winds * 2 * np.pi               
            return arm_traj


        seg_arms = task_info[args.task][args.part_index]["segments"][s]["mini-segments"][i]["arms_used"]
        
        if 'l' in seg_arms:
            best_left_path = plan_follow_traj(robot, "leftarm", left_ee_link, left_hmats, remove_winding(ds_traj[:,:7], robot.GetDOFValues(leftarm_inds)))
        else:
            best_left_path = ds_traj[:,:7]
        if 'r' in seg_arms:
            best_right_path = plan_follow_traj(robot, "rightarm", right_ee_link, right_hmats, remove_winding(ds_traj[:,7:], robot.GetDOFValues(rightarm_inds)), "leftarm", best_left_path)
        else:
            best_right_path = ds_traj[:,7:]


        left_diffs = np.abs(best_left_path[1:] - best_left_path[:-1])        
        right_diffs = np.abs(best_right_path[1:] - best_right_path[:-1])        
        print "max joint discontinuities in left arm:", left_diffs.max(), "per joint: ", left_diffs.max(axis=0)
        print "max joint discontinuities in right arm:", right_diffs.max(), "per joint: ", right_diffs.max(axis=0)

        print(colorize("look at markers in rviz. red=demo, blue=new. press enter to continue","red"))
        raw_input()

        ######################################
        ### Now view/execute the trajectory
        ######################################

        if args.mode == "openrave":
            viewer = trajoptpy.GetViewer(env)
            joint_traj = np.c_[best_left_path, best_right_path]
            # *5 factor is due to an openrave oddity
            robot.SetDOFValues([mini_segment.lgrip_angle*5, mini_segment.rgrip_angle*5], [robot.GetJoint("l_gripper_l_finger_joint").GetDOFIndex(), robot.GetJoint("r_gripper_l_finger_joint").GetDOFIndex()], False)
            for (i,row) in enumerate(joint_traj):
                print "step",i
                robot.SetActiveDOFValues(row)
                lhandle = env.drawarrow(left_ee_link.GetTransform()[:3,3], left_hmats[i][:3,3])
                rhandle = env.drawarrow(right_ee_link.GetTransform()[:3,3], right_hmats[i][:3,3])
                viewer.Idle()
        else:
            from brett2 import trajectories
            #def follow_body_traj2(pr2, bodypart2traj, times=None, wait=True, base_frame = "/base_footprint"):
            bodypart2traj = {}
            brett.lgrip.set_angle(mini_segment.lgrip_angle)
            brett.rgrip.set_angle(mini_segment.rgrip_angle)
            brett.join_all()
            sleep(.4)
            if 'l' in seg_arms:    
                bodypart2traj["l_arm"] = best_left_path
            else:
                print colorize("skipping left arm", 'yellow', bold=True, highlight=True)
            if 'r' in seg_arms:    
                bodypart2traj["r_arm"] = best_right_path                    
            else:
                print colorize("skipping right arm", 'yellow', bold=True, highlight=True)
                
            trajectories.follow_body_traj2(brett, bodypart2traj, speed_factor=.5)
