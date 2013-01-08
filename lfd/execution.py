#!/usr/bin/env python

"""
Perform deformable object manipulation task, where data is stored in some h5 file
Currently works for tying an overhand knot or folding up a laid-out towel
"""

#import argparse
#parser = argparse.ArgumentParser()
#parser.add_argument("--task", choices=["overhand_knot","figure8_knot"],default="overhand_knot")
#parser.add_argument("--human_select_demo", action="store_true")
#parser.add_argument("--prompt_before_motion", action="store_true")
#parser.add_argument("--count_steps",action="store_true")
#parser.add_argument("--hard_table",action="store_true")
#parser.add_argument("--test",action="store_true")
#args = parser.parse_args()

#import roslib; roslib.load_manifest("smach_ros")
#import smach
import lfd
from lfd import registration, trajectory_library, warping, recognition, lfd_traj
from kinematics import kinbodies
#from jds_utils.yes_or_no import yes_or_no
#import sensor_msgs.msg
#import geometry_msgs.msg as gm
import rospy
import os
import os.path as osp
from brett2 import ros_utils, PR2
from brett2.ros_utils import Marker
import numpy as np
from jds_utils import conversions
from jds_image_proc.clouds import voxel_downsample
#from jds_image_proc.alpha_shapes import get_concave_hull
import h5py
from collections import defaultdict
import yaml

DS_LENGTH = .025
DS_METHOD = "voxel"

def load_table(table_bounds):
#  table_bounds = map(float, rospy.get_param("table_bounds").split())
  kinbodies.create_box_from_bounds(Globals.pr2.env,table_bounds, name="table")

class Globals:
  pr2 = None
  rviz = None
  demos = None
  offset_trans = None # offset from tracked states to point clouds. used for transforming tracked states
  handles = []
  isinstance(pr2, PR2.PR2)
  isinstance(rviz, ros_utils.RvizWrapper)
  #if args.count_steps: stage = 0

  def __init__(self): raise

  @staticmethod
  def setup(task, table_bounds):
    if Globals.pr2 is None: 
      Globals.pr2 = PR2.PR2.create(rave_only=True)
      load_table(table_bounds)

    if Globals.rviz is None:
      Globals.rviz = ros_utils.RvizWrapper.create()

    #Globals.table_height = rospy.get_param("table_height")

    if Globals.demos is None:
      data_dir = osp.join(osp.dirname(lfd.__file__), "data")
      with open(osp.join(data_dir, "knot_demos.yaml"),"r") as fh: 
        task_info = yaml.load(fh)
      H5FILE = osp.join(data_dir, task_info[task]["db_file"])
      demos_file = h5py.File(H5FILE,"r")
      rospy.loginfo("loading demos into memory")
      Globals.demos = warping.group_to_dict(demos_file)    
      demos_file.close()
      rospy.loginfo("preprocessing demo point clouds...")
      for (_,demo) in Globals.demos.items():
        demo["cloud_xyz_ds"], ds_inds = downsample(demo["cloud_xyz"])
        #demo["cloud_xyz_ds"], ds_inds = demo["cloud_xyz"].reshape(-1, 3), np.arange(len(demo['cloud_xyz'])).reshape(-1, 1)
        demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
        demo["geodesic_dists"] = recognition.calc_geodesic_distances_downsampled_old(demo["cloud_xyz"], demo["cloud_xyz_ds"], ds_inds)
      rospy.loginfo("done")

  @staticmethod
  def destroy():
    del Globals.demos
    Globals.demos = None

def downsample(xyz):
  if DS_METHOD == "voxel":        
    xyz_ds, ds_inds = voxel_downsample(xyz, DS_LENGTH, return_inds = True)
  elif DS_METHOD == "hull":
    xyz = np.squeeze(xyz)
    _, inds = get_concave_hull(xyz[:,0:2],.05)
    xyz_ds = xyz[inds]
    ds_inds = [[i] for i in inds]    
  return xyz_ds, ds_inds

def alternate(arr1, arr2):
  assert arr1.shape == arr2.shape
  out = np.zeros((2*arr1.shape[0], arr1.shape[1]),arr1.dtype)
  out[0::2] = arr1
  out[1::2] = arr2
  return out

def calc_seg_cost(seg_name, xyz_new_ds, dists_new):
  candidate_demo = Globals.demos[seg_name]
  xyz_demo_ds = np.squeeze(candidate_demo["cloud_xyz_ds"])
  dists_demo = candidate_demo["geodesic_dists"]
  cost = recognition.calc_match_score(xyz_demo_ds, xyz_new_ds, dists0 = dists_demo, dists1 = dists_new)
  #cost = recognition.calc_match_score(xyz_new_ds, xyz_demo_ds, dists0 = dists_new, dists1 = dists_demo)
  print "seg_name: %s. cost: %s"%(seg_name, cost)
  return cost, seg_name

def clipinplace(x,lo,hi):
  np.clip(x,lo,hi,out=x)

#def look_at_object(xyz_in):
#  xyz = xyz_in.reshape(-1,3)
#  xyz = ros_utils.transform_points(xyz, Globals.pr2.tf_listener, "base_footprint", msg.header.frame_id)
#  return ('success', xyz)

def rviz_draw_points(pts, **kwargs):
  pose_array = conversions.array_to_pose_array(np.squeeze(pts), "base_footprint")
  Globals.handles.append(Globals.rviz.draw_curve(pose_array, **kwargs))

def get_demos():
  return Globals.demos

def select_trajectory(points, curr_robot_joint_vals, curr_step):
  """
  - lookup closest trajectory from database
  - if it's a terminal state, we're done
  - warp it based on the current rope
  returns: done, not_done, failure
  """
  xyz_new = np.squeeze(np.asarray(points))
  #if args.obj == "cloth": xyz_new = voxel_downsample(xyz_new, .025)
  xyz_new_ds, ds_inds = downsample(xyz_new)
#  xyz_new_ds, ds_inds = xyz_new.reshape(-1,3), np.arange(0, len(xyz_new)).reshape(-1, 1)
  dists_new = recognition.calc_geodesic_distances_downsampled_old(xyz_new,xyz_new_ds, ds_inds)
  candidate_demo_names = Globals.demos.keys()

  # HACK: never choose the done step on the first step
# print 'curr step', curr_step
# if curr_step == 0:
#   filtered_demo_names = []
#   for n in candidate_demo_names:
#     if not Globals.demos[n]['done']:
#       filtered_demo_names.append(n)
#   candidate_demo_names = filtered_demo_names

  from joblib import parallel
  #costs_names = parallel.Parallel(n_jobs = 4)(parallel.delayed(calc_seg_cost)(seg_name, xyz_new_ds, dists_new) for seg_name in candidate_demo_names)
  costs_names = [calc_seg_cost(seg_name, xyz_new_ds, dists_new) for seg_name in sorted(candidate_demo_names)]
  _, best_name = min(costs_names)
  print "choices: ", candidate_demo_names
  best_name = raw_input("type name of trajectory you want to use\n")
  rospy.loginfo('costs_names %s', costs_names)

  #matcher = recognition.CombinedNNMatcher(recognition.DataSet.LoadFromDict(Globals.demos), [recognition.GeodesicDistMatcher, recognition.ShapeContextMatcher], [1, 0.1])
  #best_name, best_cost = matcher.match(xyz_new)

  best_demo = Globals.demos[best_name]
  if best_demo["done"]:
    rospy.loginfo("best demo was a 'done' state")
    return {'status': 'success'}
  rospy.loginfo("best segment name: %s", best_name)
  xyz_demo_ds = best_demo["cloud_xyz_ds"]

#  print 'arms used', best_demo['arms_used']
#  overlap_ctl_pts = []
#  grabbing_pts = []
#  for lr in 'lr':
#    # look at points around gripper when grabbing
#    grabbing = map(bool, list(best_demo["%s_gripper_joint"%lr] < .07))
#    grabbing_pts.extend([p for i, p in enumerate(best_demo["%s_gripper_l_finger_tip_link"%lr]["position"]) if grabbing[i] and (i == 0 or not grabbing[i-1])])
#    grabbing_pts.extend([p for i, p in enumerate(best_demo["%s_gripper_r_finger_tip_link"%lr]["position"]) if grabbing[i] and (i == 0 or not grabbing[i-1])])
#  overlap_ctl_pts = [p for p in xyz_demo_ds if any(np.linalg.norm(p - g) < 0.1 for g in grabbing_pts)]
#  overlap_ctl_pts = xyz_demo_ds
  #rviz_draw_points(overlap_ctl_pts,rgba=(1,1,1,1),type=Marker.CUBE_LIST)
#  rviz_draw_points(grabbing_pts,rgba=(.5,.5,.5,1),type=Marker.CUBE_LIST)
  n_iter = 101
  #warping_map = registration.tps_rpm_with_overlap_control(xyz_demo_ds, xyz_new_ds, overlap_ctl_pts, reg_init=1,reg_final=.01,n_iter=n_iter,verbose=False, plotting=20)
  warping_map,info = registration.tps_rpm(xyz_demo_ds, xyz_new_ds, reg_init=1,reg_final=.01,n_iter=n_iter,verbose=False, plotting=20,return_full=True)  

  from lfd import tps
  import scipy.spatial.distance as ssd
  f = warping_map
  pts_grip = []
  for lr in "lr":
    if best_demo["arms_used"] in ["b", lr]:
      pts_grip.extend(best_demo["%s_gripper_tool_frame"%lr]["position"])
  pts_grip = np.array(pts_grip)
  dist_to_rope = ssd.cdist(pts_grip, xyz_demo_ds).min(axis=1)
  pts_grip_near_rope = pts_grip[dist_to_rope < .04,:]
  pts_rigid = voxel_downsample(pts_grip_near_rope, .01)

  registration.Globals.handles = []
  f.lin_ag, f.trans_g, f.w_ng, f.x_na = tps.tps_nr_fit_enhanced(info["x_Nd"], info["targ_Nd"], 0.01, pts_rigid, 0.001, method="newton", plotting=5)
  
  #if plotting:
    #plot_orig_and_warped_clouds(f.transform_points, x_nd, y_md)   
    #targ_pose_array = conversions.array_to_pose_array(targ_Nd, 'base_footprint')
    #Globals.handles.append(Globals.rviz.draw_curve(targ_pose_array,rgba=(1,1,0,1),type=Marker.CUBE_LIST))

  #raw_input('Press enter to continue:')
  #################### Generate new trajectory ##################

  #### Plot original and warped point clouds #######
  #orig_pose_array = conversions.array_to_pose_array(np.squeeze(best_demo["cloud_xyz_ds"]), "base_footprint")
  #warped_pose_array = conversions.array_to_pose_array(warping_map.transform_points(np.squeeze(best_demo["cloud_xyz_ds"])), "base_footprint")
  #Globals.handles.append(Globals.rviz.draw_curve(orig_pose_array,rgba=(1,0,0,1),id=19024,type=Marker.CUBE_LIST))
  #Globals.handles.append(Globals.rviz.draw_curve(warped_pose_array,rgba=(0,1,0,1),id=2983,type=Marker.CUBE_LIST))

  #### Plot grid ########
  #mins = np.squeeze(best_demo["cloud_xyz"]).min(axis=0)
  #maxes = np.squeeze(best_demo["cloud_xyz"]).max(axis=0)
  #mins[2] -= .1
  #maxes[2] += .1
  #grid_handle = warping.draw_grid(Globals.rviz, warping_map.transform_points, mins, maxes, 'base_footprint')
  #Globals.handles.append(grid_handle)

  #### Actually generate the trajectory ###########
  warped_demo = warping.transform_demo_with_fingertips(warping_map, best_demo)

  Globals.pr2.update_rave_without_ros(curr_robot_joint_vals)
  trajectory = {}
  trajectory['seg_name'] = best_name
  trajectory['demo'] = best_demo
  if 'tracked_states' in best_demo:
    trajectory['orig_tracked_states'] = best_demo['tracked_states']
    trajectory['tracked_states'], Globals.offset_trans = warping.transform_tracked_states(warping_map, best_demo, Globals.offset_trans)

  for lr in "lr":
    leftright = {"l":"left","r":"right"}[lr]
    if best_demo["arms_used"] in [lr, "b"]:
      #if args.hard_table:
      #    clipinplace(warped_demo["l_gripper_tool_frame"]["position"][:,2],Globals.table_height+.032,np.inf)
      #    clipinplace(warped_demo["r_gripper_tool_frame"]["position"][:,2],Globals.table_height+.032,np.inf)

      #arm_traj, feas_inds = lfd_traj.make_joint_traj(
      #  warped_demo["%s_gripper_tool_frame"%lr]["position"],
      #  warped_demo["%s_gripper_tool_frame"%lr]["orientation"],
      #  best_demo["%sarm"%leftright],
      #  Globals.pr2.robot.GetManipulator("%sarm"%leftright),
      #  "base_footprint","%s_gripper_tool_frame"%lr,
      #  1+2+16
      #)
      arm_traj, feas_inds = lfd_traj.make_joint_traj_by_graph_search(
        warped_demo["%s_gripper_tool_frame"%lr]["position"],
        warped_demo["%s_gripper_tool_frame"%lr]["orientation"],
        Globals.pr2.robot.GetManipulator("%sarm"%leftright),
        "%s_gripper_tool_frame"%lr
      )
      if len(feas_inds) == 0: return {'status': "failure"}
      trajectory["%s_arm"%lr] = arm_traj
      trajectory['steps'] = len(arm_traj)
      rospy.loginfo("left arm: %i of %i points feasible", len(feas_inds), len(arm_traj))
      trajectory["%s_grab"%lr] = map(bool, list(best_demo["%s_gripper_joint"%lr] < .02))
      trajectory["%s_gripper"%lr] = warped_demo["%s_gripper_joint"%lr]
      trajectory["%s_gripper"%lr][trajectory["%s_grab"%lr]] = 0
      # plot warped trajectory
      Globals.handles.append(Globals.rviz.draw_curve(conversions.array_to_pose_array(alternate(warped_demo["%s_gripper_l_finger_tip_link"%lr]["position"],warped_demo["%s_gripper_r_finger_tip_link"%lr]["position"]), "base_footprint"), width=.001, rgba = (1,0,1,.4),type=Marker.LINE_LIST))
      # plot original trajectory
      Globals.handles.append(Globals.rviz.draw_curve(conversions.array_to_pose_array(alternate(best_demo["%s_gripper_l_finger_tip_link"%lr]["position"],best_demo["%s_gripper_r_finger_tip_link"%lr]["position"]), "base_footprint"), width=.001, rgba = (0,1,1,.4),type=Marker.LINE_LIST))
  #raw_input('Press enter to continue:')

  return {'status': 'not_done', 'trajectory': trajectory}

def init(task, table_bounds):
  Globals.setup(task, table_bounds)
  if rospy.get_name() == '/unnamed':
    rospy.init_node("tie_knot",disable_signals=True)
