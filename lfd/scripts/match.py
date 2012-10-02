#!/usr/bin/env python

import numpy as np

import lfd
import os
from lfd import recognition, warping, registration
import os.path as osp
import h5py
import yaml

PARALLEL = False

class DebugDrawing:
  enabled = False
  lines = []
  @staticmethod
  def clear():
    DebugDrawing.lines = []

def downsample(xyz):
  from jds_image_proc.clouds import voxel_downsample
  DS_LENGTH = .025
  xyz_ds, ds_inds = voxel_downsample(xyz, DS_LENGTH, return_inds = True)
  return xyz_ds, ds_inds


class DataSet(object):
  def __init__(self): self.data = {}
  def keys(self): return self.data.keys()
  def items(self): return self.data.items()
  def set(self, key, val): self.data[key] = val
  def get(self, key): return self.data[key]
  def __getitem__(self, key): return self.data[key]
  def __setitem__(self, key, value): self.data[key] = value

  @staticmethod
  def LoadFromTaskDemos(taskname, demos_list_file='knot_demos.yaml'):
    # load demos
    data_dir = osp.join(osp.dirname(lfd.__file__), "data")
    with open(osp.join(data_dir, demos_list_file),"r") as fh: 
      task_info = yaml.load(fh)
    H5FILE = osp.join(data_dir, task_info[taskname]["db_file"])
    demos_file = h5py.File(H5FILE,"r")
    demos = warping.group_to_dict(demos_file)    
    demos_file.close()

    ds = DataSet()
    for name, demo in demos.items():
      ds[name] = demo
    return ds


class BasicMatcher(object):
  def __init__(self, dataset):
    assert isinstance(dataset, DataSet)
    self.dataset = dataset
  def get_dataset(self): return self.dataset
  def get_name(self): raise NotImplementedError
  def match(self, xyz): raise NotImplementedError


class NearestNeighborMatcher(BasicMatcher):
  def match(self, xyz):
    input = self.preprocess_input(xyz)
    costs_names = [(self.calc_cost(self.dataset[seg_name], input), seg_name) for seg_name in self.dataset.keys()]
    for cost, name in sorted(costs_names):
      print 'Segment %s: cost %f' % (name, cost)
    best_cost, best_name = min(costs_names)
    return best_name, best_cost

  def preprocess_input(self, input):
    return input

  def calc_cost(self, dataset_item, preprocessed_input):
    raise NotImplementedError


class CombinedNNMatcher(NearestNeighborMatcher):
  def __init__(self, dataset, nn_matchers, weights):
    NearestNeighborMatcher.__init__(self, dataset)
    assert len(nn_matchers) == len(weights)
    self.nn_matchers = [matcher_class(dataset) for matcher_class in nn_matchers]
    self.weights = weights

  def match(self, xyz):
    seg_names = sorted(self.dataset.keys())
    total_costs = np.zeros(len(seg_names))
    matcher_costs = []
    for i, matcher in enumerate(self.nn_matchers):
      input = matcher.preprocess_input(xyz)
      costs = [matcher.calc_cost(self.dataset[seg_name], input) for seg_name in seg_names]
      matcher_costs.append(costs)
      total_costs += [self.weights[i] * c for c in costs]
    sorted_total_costs = sorted((total_cost, i) for i, total_cost in enumerate(total_costs))
    for total_cost, i in sorted_total_costs:
      comb_str = ' + '.join('%.2f*%f' % (self.weights[j], matcher_cost[i]) for j, matcher_cost in enumerate(matcher_costs))
      print 'Segment %s: cost %f = %s' % (seg_names[i], total_cost, comb_str)
    best_cost, best_idx = sorted_total_costs[0]
    return seg_names[best_idx], best_cost


class GeodesicDistMatcher(NearestNeighborMatcher):
  def __init__(self, dataset):
    BasicMatcher.__init__(self, dataset)
    # preprocess dataset
    for _, demo in self.dataset.items():
      demo["cloud_xyz_ds"], ds_inds = downsample(demo["cloud_xyz"])
      demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
      demo["geodesic_dists"] = recognition.calc_geodesic_distances_downsampled_old(demo["cloud_xyz"], demo["cloud_xyz_ds"], ds_inds)

  def get_name(self):
    return 'GeodesicDistMatcher'

  def preprocess_input(self, xyz_new):
    xyz_new_ds, ds_inds = downsample(xyz_new)
    dists_new = recognition.calc_geodesic_distances_downsampled_old(xyz_new, xyz_new_ds, ds_inds)
    return xyz_new_ds, dists_new

  def calc_cost(self, dataset_item, preprocessed_input):
    xyz_new_ds, dists_new = preprocessed_input
    xyz_demo_ds = np.squeeze(dataset_item["cloud_xyz_ds"])
    dists_demo = dataset_item['geodesic_dists']
    cost = recognition.calc_match_score(xyz_demo_ds, xyz_new_ds, dists0=dists_demo, dists1=dists_new)
    return cost


class ShapeContextMatcher(NearestNeighborMatcher):
  def __init__(self, dataset):
    BasicMatcher.__init__(self, dataset)
    for _, demo in self.dataset.items():
      demo["cloud_xyz_ds"], ds_inds = downsample(demo["cloud_xyz"])
      demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
      demo['shape_context_hists'] = ShapeContextMatcher.calc_shape_context_hists(demo['cloud_xyz_ds'])

  def get_name(self):
    return 'ShapeContextMatcher'

  def preprocess_input(self, xyz_new):
    xyz_new_ds, ds_inds = downsample(xyz_new)
    hists_new = ShapeContextMatcher.calc_shape_context_hists(xyz_new_ds)
    return xyz_new_ds, hists_new

  def calc_cost(self, dataset_item, preprocessed_input):
    xyz_new_ds, hists_new = preprocessed_input
    xyz_demo_ds, hists_demo = np.squeeze(dataset_item["cloud_xyz_ds"]), dataset_item['shape_context_hists']
    f = registration.tps_rpm(xyz_demo_ds, xyz_new_ds, plotting=False,reg_init=1,reg_final=.1,n_iter=21, verbose=False)
    partners = f.corr.argmax(axis=1)
    return ShapeContextMatcher.compare_hists_with_permutation(hists_demo, hists_new, partners)

  @staticmethod
  def calc_shape_context_hists(xy, logr_bins=10, theta_bins=20, normalize_angles=False, pca_radius=0.1):
    xy = xy[:,:2]
    npoints = xy.shape[0]

    if normalize_angles:
      from scipy import spatial
      princomps = np.zeros((npoints, 2))
      princomp_angles = np.zeros((npoints, 1))
      kdtree = spatial.KDTree(xy)
      for i in xrange(npoints):
        ball = xy[kdtree.query_ball_point(xy[i], pca_radius)] - xy[i]
        pc = np.linalg.svd(ball)[2][0]
        princomps[i] = pc
        princomp_angles[i] = np.arctan2(pc[1], pc[0])
        if DebugDrawing.enabled:
          DebugDrawing.lines.append((xy[i], xy[i] + princomps[i]/np.linalg.norm(princomps[i])*0.1))

    def calc_one_hist(c):
      lp = np.empty_like(xy)
      for i in xrange(npoints):
        dp = xy[i] - c
        logdist = np.log(np.linalg.norm(dp) + 1e-6)
        angle = np.arctan2(dp[1], dp[0])
        if normalize_angles:
          # look at angle relative to tangent (first principal component)
          angle -= princomp_angles[i]
        lp[i] = logdist, angle

      if DebugDrawing.enabled:
        draw_comparison(xy, lp)

      return np.histogram2d( \
        lp[:,0], lp[:,1], \
        bins=(logr_bins, theta_bins), \
        range=((-1e6, 3), (0, 2.*np.pi)) \
      )[0]

    hists = np.zeros((npoints, logr_bins, theta_bins))
    for k in xrange(npoints):
      hists[k] = calc_one_hist(xy[k])
    return hists

  @staticmethod
  def compare_hists_with_permutation(h1, h2, perm):
    h2_permed = h2[perm]
    assert h2_permed.shape == h1.shape
    # chi-squared test statistic
    return 0.5*((h2_permed - h1)**2 / (h2_permed + h1 + 1)).sum() / h1.shape[0]


def draw_comparison(left_cloud, right_cloud):
  from traits.api import HasTraits, Instance, Button, on_trait_change
  from traitsui.api import View, Item, HSplit, Group
  from mayavi import mlab
  from mayavi.core.ui.api import MlabSceneModel, SceneEditor

  class ComparisonDialog(HasTraits):
    scene1 = Instance(MlabSceneModel, ())
    scene2 = Instance(MlabSceneModel, ())
    view = View(
      HSplit(
        Group(
          Item(
            'scene1',
            editor=SceneEditor(),
            height=400,
            width=400,
            show_label=False,
          ),
          show_labels=False,
        ),
        Group(
          Item(
            'scene2',
            editor=SceneEditor(),
            height=400,
            width=400,
            show_label=False
          ),
          show_labels=False,
        ),
      ),
      resizable=True,
    )

    @on_trait_change('scene1.activated')
    def redraw_scene1(self):
      self.redraw_scene(self.scene1, left_cloud)
      for line in DebugDrawing.lines:
        mlab.plot3d([line[0][0], line[1][0]], [line[0][1], line[1][1]], [0, 0], tube_radius=None, figure=self.scene1.mayavi_scene)

    @on_trait_change('scene2.activated')
    def redraw_scene2(self):
      self.redraw_scene(self.scene2, right_cloud)

    def redraw_scene(self, scene, input_xyz):
      mlab.clf(figure=scene.mayavi_scene)
      if input_xyz.shape[1] == 2:
        x, y = input_xyz.transpose()
        z = np.zeros(x.shape)
      elif input_xyz.shape[1] == 3:
        x, y, z = input_xyz.transpose()
      else:
        raise NotImplementedError
      mlab.points3d(x, y, z, mode='point', figure=scene.mayavi_scene)

  m = ComparisonDialog()
  m.configure_traits()

def select_from_list(lst, prompt='Choose from the following options: '):
    strlist = [str(item) for item in lst]
    while True:
      print prompt + " ".join("(%s)"%item for item in strlist)
      resp = raw_input("> ")
      if resp not in strlist:
        print "invalid response. try again."
      else:
        return lst[strlist.index(resp)]            

def main():
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--dataset', default='overhand_knot', help='name of dataset')
  parser.add_argument('--method', choices=['geodesic_dist', 'shape_context', 'geodesic_dist+shape_context'], default='geodesic_dist', help='matching algorithm')
  parser.add_argument('--input_mode', choices=['from_dataset', 'kinect'], default='kinect', help='input cloud acquisition method')
  parser.add_argument('--cloud_topic', default='/preprocessor/kinect1/points', help='ros topic for kinect input mode')
  args = parser.parse_args()

  dataset = DataSet.LoadFromTaskDemos(args.dataset)

  # read input to match
  input_xyz = None
  if args.input_mode == 'from_dataset':
    key = select_from_list(sorted(dataset.keys()))
    input_xyz = np.squeeze(np.asarray(dataset[key]['cloud_xyz']))
  elif args.input_mode == 'kinect':
    import rospy
    from brett2 import ros_utils
    import sensor_msgs
    import time
    if rospy.get_name() == '/unnamed':
      rospy.init_node('matcher', disable_signals=True)
    msg = rospy.wait_for_message(args.cloud_topic, sensor_msgs.msg.PointCloud2)
    xyz, rgb = ros_utils.pc2xyzrgb(msg)
    xyz = xyz.reshape(-1, 3)
    tf_listener = ros_utils.get_tf_listener()
    time.sleep(1) # so tf works
    input_xyz = ros_utils.transform_points(xyz, tf_listener, "ground", msg.header.frame_id)
  else:
    raise NotImplementedError

  # create the matcher
  matcher = None
  if args.method == 'geodesic_dist':
    matcher = GeodesicDistMatcher(dataset)
  elif args.method == 'shape_context':
    matcher = ShapeContextMatcher(dataset)
  elif args.method == 'geodesic_dist+shape_context':
    matcher = CombinedNNMatcher(dataset, [GeodesicDistMatcher, ShapeContextMatcher], [1, 0.1])
  else:
    raise NotImplementedError

  # do the matching
  best_match_name, best_match_cost = matcher.match(input_xyz)
  print 'Best match name:', best_match_name, 'with cost:', best_match_cost
  draw_comparison(left_cloud=input_xyz, right_cloud=dataset[best_match_name]['cloud_xyz'])

if __name__ == '__main__':
  main()
