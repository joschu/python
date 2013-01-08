#!/usr/bin/env python

import numpy as np
from brett2 import ros_utils
from brett2.ros_utils import Marker
import jds_utils
from jds_utils import conversions
import rospy
import match
import lfd, yaml, os, h5py
from lfd import warping, registration, recognition
import multiprocessing as mp

class Globals:
  rviz = None
  handles = []
  tf_listener = None
  isinstance(rviz, ros_utils.RvizWrapper)
  #isinstance(tf_listener, ...)
  def __init__(self): raise
  @staticmethod
  def setup():
    if Globals.rviz is None: Globals.rviz = ros_utils.RvizWrapper.create()
    if Globals.tf_listener is None: Globals.tf_listener = ros_utils.get_tf_listener()

def select_from_list(lst, prompt='Choose from the following options: '):
    strlist = [str(item) for item in lst]
    while True:
      print prompt + " ".join("(%s)"%item for item in strlist)
      resp = raw_input("> ")
      if resp not in strlist:
        print "invalid response. try again."
      else:
        return lst[strlist.index(resp)]            

def load_demo(taskname='overhand_knot', demos_list_file='knot_demos.yaml', seg_name=None):
  data_dir = os.path.join(os.path.dirname(lfd.__file__), "data")
  with open(os.path.join(data_dir, demos_list_file),"r") as fh: 
    task_info = yaml.load(fh)
  H5FILE = os.path.join(data_dir, task_info[taskname]["db_file"])
  demos_file = h5py.File(H5FILE,"r")
  demos = warping.group_to_dict(demos_file)    
  demos_file.close()
  if seg_name is None:
    seg_name = select_from_list(sorted(demos.keys()))
  demo = demos[seg_name]
  demo["cloud_xyz_ds"], ds_inds = recognition.downsample(demo["cloud_xyz"])
  demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
  return demo

def load_cloud_from_sensor(input_topic, frame):
  import sensor_msgs.msg
  msg = rospy.wait_for_message(input_topic, sensor_msgs.msg.PointCloud2)
  xyz, rgb = ros_utils.pc2xyzrgb(msg)
  xyz = xyz.reshape(-1,3)
  xyz = ros_utils.transform_points(xyz, Globals.tf_listener, frame, msg.header.frame_id)
  return xyz

def fit_warp(args, cloud1, cloud2, set_warp_params=None):
  def read_warp_params(params):
    return ([float(p.strip()) for p in set_warp_params.strip().split()])

  if args.warp_type == 'nonrigid':
    f = registration.tps_rpm(cloud1, cloud2, plotting=False,reg_init=1,reg_final=.1,n_iter=21, verbose=False)
  elif args.warp_type == 'rigid3d':
    f = registration.Rigid3d()
    if set_warp_params is not None:
      f.set_params(read_warp_params(set_warp_params))
    else:
      f.fit(cloud1, cloud2)
  elif args.warp_type == 'translation3d':
    f = registration.Translation3d()
    if set_warp_params is not None:
      f.set_params(read_warp_params(set_warp_params))
    else:
      f.fit(cloud1, cloud2)
  else:
    raise NotImplementedError
  return f

def warping_loop(args, take_snapshot, quit):
  if rospy.get_name() == '/unnamed':
    rospy.init_node('publish_single_cloud', disable_signals=True)
  Globals.setup()

  demo = load_demo(args.taskname, args.demos_list_file, args.seg_name)

  imarker_server = None
  if args.warp_type == 'rigid_interactive':
    from interactive_markers.interactive_marker_server import *

    init_pos = demo['cloud_xyz'].mean(axis=0)[:3]
    marker_6dof = utils_rviz.make_interactive_marker_6dof(init_pos, args.frame)
    imarker_server = InteractiveMarkerServer('demo_pose_control')
    def callback_fn(feedback):
      xyz = feedback.pose.position.x, feedback.pose.position.y, feedback.pose.position.z
    imarker_server.insert(marker_6dof, callback_fn)
    imarker_server.applyChanges()

  # align demo to test in a loop
  #prev_params = None
  keep_warping = True
  num_saved = 0

  while not quit.value:
    xyz_new = load_cloud_from_sensor(args.input_topic, args.frame)
    if xyz_new.size == 0: continue

    xyz_demo_ds = demo['cloud_xyz_ds']
    xyz_new_ds = recognition.downsample(xyz_new)[0]

    if keep_warping:
      f = fit_warp(args, xyz_demo_ds, xyz_new_ds, set_warp_params=args.set_warp_params)
      if hasattr(f, 'get_params') and args.set_warp_params is None:
        print 'fitted warp params: "%s"' % (' '.join(map(str, f.get_params())), )
      if args.warp_once_only:
        keep_warping = False

    if args.show_shape_contexts:
      shape_context_costs = recognition.match_and_calc_shape_context(xyz_demo_ds, xyz_new_ds, normalize_costs=True)
      colors = [(c, 1-c, 0, 1) for c in shape_context_costs]
    else:
      import itertools
      colors = itertools.repeat((1, 0, 0, 1))

    warped_pose_array = conversions.array_to_pose_array(f.transform_points(xyz_demo_ds), args.frame)
    Globals.handles = []
    import geometry_msgs.msg as gm
    for p, rgba in zip(warped_pose_array.poses, colors):
      ps = gm.PoseStamped()
      ps.header = warped_pose_array.header
      ps.pose = p
      Globals.handles.append(Globals.rviz.draw_marker(ps, scale=(.01, .01, .01), rgba=rgba))

    if take_snapshot.value:
      filename = '%s%04d' % (args.save_prefix, num_saved)
      print 'Saving snapshot to %s' % (filename,)
      take_snapshot.value = False

      finv = fit_warp(args, xyz_new_ds, xyz_demo_ds, set_warp_params=args.set_inv_warp_params)
      xyz_new_ds_out = finv.transform_points(xyz_new_ds)
      xyz_new_out = finv.transform_points(xyz_new)

      np.savez(filename, cloud_xyz=xyz_new_out, cloud_xyz_ds=xyz_new_ds_out)
      num_saved += 1
      print 'Done saving snapshot. (used inverse transformation "%s")' % (' '.join(map(str, finv.get_params())), )

def main():
  import argparse
  parser = argparse.ArgumentParser(description='Tool for aligning a demo to a live point cloud')
  parser.add_argument('--demos_list_file', default='knot_demos.yaml')
  parser.add_argument('--taskname', default='overhand_knot')
  parser.add_argument('--seg_name', default=None)
  parser.add_argument('--warp_type', choices=['nonrigid', 'rigid3d', 'translation3d' 'rigid_interactive'], default='rigid3d')
  parser.add_argument('--set_warp_params', default=None)
  parser.add_argument('--set_inv_warp_params', default=None)
  parser.add_argument('--frame', default='camera_rgb_frame')
  parser.add_argument('--input_topic', default='/preprocessor/kinect1/points')
  parser.add_argument('--show_shape_contexts', action='store_true')
  parser.add_argument('--warp_once_only', action='store_true')
  parser.add_argument('--save_prefix', default='/tmp/xyz_')
  args = parser.parse_args()

  take_snapshot = mp.Value('b', False)
  quit = mp.Value('b', False)
  loop_proc = mp.Process(target=warping_loop, args=(args, take_snapshot, quit))
  loop_proc.start()

  while True:
    user_input = raw_input('([s]napshot/[h]elp/[q]uit)> ').strip()
    if user_input == 's' or user_input == 'snapshot':
      take_snapshot.value = True
    elif user_input == 'q' or user_input == 'quit':
      quit.value = True
      break
    elif user_input == 'h' or user_input == 'help':
      print "'snapshot' saves the current cloud to a file determined by save_prefix\n'help' prints this message\n'quit' quits"
    elif user_input == '':
      continue
    else:
      print 'unrecognized input: ', user_input

  loop_proc.join()


if __name__ == '__main__':
  main()
