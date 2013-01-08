import numpy as np

def select_from_list(lst, prompt='Choose from the following options: '):
    strlist = [str(item) for item in lst]
    while True:
      print prompt + " ".join("(%s)"%item for item in strlist)
      resp = raw_input("> ")
      if resp not in strlist:
        print "invalid response. try again."
      else:
        return lst[strlist.index(resp)]            

def read_from_dataset(demos_list_file, dataset_name, seg_name):
  '''format: dataset:knot_demos.yaml:overhand_knot:00.01'''
  import recognition
  dataset = recognition.DataSet.LoadFromTaskDemos(dataset_name, demos_list_file)
  if seg_name is None:
    seg_name = select_from_list(sorted(dataset.keys()))
  return np.squeeze(np.asarray(dataset[seg_name]['cloud_xyz']))

def read_from_ros(cloud_topic, frame='ground'):
  '''format: ros:/my/topic/points(:frame)'''
  import rospy
  from brett2 import ros_utils
  import sensor_msgs
  import time
  if rospy.get_name() == '/unnamed':
    rospy.init_node('cloud_reader', disable_signals=True)
  msg = rospy.wait_for_message(cloud_topic, sensor_msgs.msg.PointCloud2)
  xyz, rgb = ros_utils.pc2xyzrgb(msg)
  xyz = xyz.reshape(-1, 3)
  tf_listener = ros_utils.get_tf_listener()
  return ros_utils.transform_points(xyz, tf_listener, frame, msg.header.frame_id)

def read_from_npz(filename):
  '''format: file:/home/blah/file.npz'''
  return np.load(filename)['cloud_xyz']

def write_cloud(out_file, cloud):
  np.savez(out_file, cloud_xyz=out_file)

def read_from_rsrc(rsrc):
  parts = rsrc.split(':')
  type, args = parts[0], parts[1:]
  if type == 'dataset':
    return read_from_dataset(*args)
  elif type == 'ros':
    return read_from_ros(*args)
  elif type == 'file':
    return read_from_npz(*args)
  else:
    raise NotImplementedError('unknown resource type ' + type)

#def main():
#  import argparse
#  parser = argparse.ArgumentParser()
#  parser.add_argument('--input_mode', choices=['from_dataset', 'kinect'], default='kinect', help='input cloud acquisition method')
#  parser.add_argument('--dataset', default='overhand_knot', help='name of dataset for input_mode=from_dataset')
#  parser.add_argument('--cloud_topic', default='/preprocessor/kinect1/points', help='ros topic for input_mode=kinect')
#  parser.add_argument('out', help='output file')
#  args = parser.parse_args()
#
#  if args.input_mode == 'from_dataset':
#    write_cloud(args.out, read_from_dataset(args.dataset, None))
#  elif args.input_mode == 'kinect':
#    write_cloud(args.out, read_from_ros(args.cloud_topic))
#  else:
#    raise NotImplementedError
#
#if __name__ == '__main__':
#  main()
