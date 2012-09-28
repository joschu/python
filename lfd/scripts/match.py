import numpy as np

import lfd
import os
from lfd import recognition, warping, registration
import os.path as osp
import h5py
import yaml

PARALLEL = False

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

class GeodesicDistMatcher(BasicMatcher):
  def __init__(self, dataset):
    BasicMatcher.__init__(self, dataset)
    # preprocess dataset
    for _, demo in self.dataset.items():
      demo["cloud_xyz_ds"], ds_inds = downsample(demo["cloud_xyz"])
      demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
      demo["geodesic_dists"] = recognition.calc_geodesic_distances_downsampled_old(demo["cloud_xyz"], demo["cloud_xyz_ds"], ds_inds)

  def get_name(self):
    return 'GeodesicDistMatcher'

  def match(self, xyz_new):
    # nearest neighbor based on geodesic distances
    def _calc_seg_cost(dataset_item, xyz_new_ds, dists_new):
      xyz_demo_ds = np.squeeze(dataset_item["cloud_xyz_ds"])
      dists_demo = dataset_item['geodesic_dists']
      cost = recognition.calc_match_score(xyz_demo_ds, xyz_new_ds, dists0=dists_demo, dists1=dists_new)
      return cost
    xyz_new_ds, ds_inds = downsample(xyz_new)
    dists_new = recognition.calc_geodesic_distances_downsampled_old(xyz_new, xyz_new_ds, ds_inds)
    costs_names = [(_calc_seg_cost(self.dataset[seg_name], xyz_new_ds, dists_new), seg_name) for seg_name in self.dataset.keys()]
    for cost, name in sorted(costs_names):
      print 'Segment %s: cost %f' % (name, cost)
    best_cost, best_name = min(costs_names)
    return best_name, best_cost

def calc_shape_context_hists(xy, logr_bins=10, theta_bins=20):
  xy = xy[:,:2]
  def calc_one_hist(c):
#   xy_cv, lp_cv = cv.fromarray(xy), cv.fromarray(xy)
#   print 'center', c, list(c), (c[0], c[1])
#   cv.LogPolar(xy_cv, lp_cv, center=map(float, list(c)), M=1)
    lp = np.empty_like(xy)
    for i in xrange(xy.shape[0]):
      p = xy[i]
      dist, angle = np.linalg.norm(p-c), np.arctan2(p[1]-c[1], p[0]-c[0])
      lp[i] = np.log(dist + 1e-6), angle
    #draw_comparison(xy, lp)
    hist, _ = np.histogramdd(lp, (logr_bins, theta_bins))
    return hist

  hists = np.zeros((xy.shape[0], logr_bins, theta_bins))
  for k in xrange(xy.shape[0]):
    hists[k] = calc_one_hist(xy[k])
  return hists

def compare_hists_with_permutation(h1, h2, perm):
  h2_permed = h2[perm]
  assert h2_permed.shape == h1.shape
  # chi-squared test statistic
  return 0.5*((h2_permed - h1)**2 / (h2_permed + h1 + 1)).sum() / h1.shape[0]

class ShapeContextMatcher(BasicMatcher):
  def __init__(self, dataset):
    BasicMatcher.__init__(self, dataset)
    for _, demo in self.dataset.items():
      demo["cloud_xyz_ds"], ds_inds = downsample(demo["cloud_xyz"])
      demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
      demo['shape_context_hists'] = calc_shape_context_hists(demo['cloud_xyz_ds'])

  def get_name(self):
    return 'ShapeContextMatcher'

  def match(self, xyz_new):
    def _calc_seg_cost(dataset_item, xyz_new_ds, hists_new):
      xyz_demo_ds = np.squeeze(dataset_item["cloud_xyz_ds"])
      hists_demo = dataset_item['shape_context_hists']

      f = registration.tps_rpm(xyz_demo_ds, xyz_new_ds, plotting=False,reg_init=1,reg_final=.1,n_iter=21, verbose=False)
      partners = f.corr.argmax(axis=1)

      return compare_hists_with_permutation(hists_demo, hists_new, partners)

    xyz_new_ds, ds_inds = downsample(xyz_new)
    hists_new = calc_shape_context_hists(xyz_new_ds)
    #dists_new = recognition.calc_geodesic_distances_downsampled_old(xyz_new, xyz_new_ds, ds_inds)
    costs_names = [(_calc_seg_cost(self.dataset[seg_name], xyz_new_ds, hists_new), seg_name) for seg_name in self.dataset.keys()]
    for cost, name in sorted(costs_names):
      print 'Segment %s: cost %f' % (name, cost)
    best_cost, best_name = min(costs_names)
    return best_name, best_cost

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
  parser.add_argument('--method', choices=['geodesic_dist', 'shape_context'], default='geodesic_dist', help='matching algorithm')
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
    if rospy.get_name() == '/unnamed':
      rospy.init_node('matcher', disable_signals=True)
    msg = rospy.wait_for_message(args.cloud_topic, sensor_msgs.msg.PointCloud2)
    xyz, rgb = ros_utils.pc2xyzrgb(msg)
    xyz = xyz.reshape(-1, 3)
    input_xyz = ros_utils.transform_points(xyz, ros_utils.get_tf_listener(), "ground", msg.header.frame_id)
  else:
    raise NotImplementedError

  # create the matcher
  matcher = None
  if args.method == 'geodesic_dist':
    matcher = GeodesicDistMatcher(dataset)
  elif args.method == 'shape_context':
    matcher = ShapeContextMatcher(dataset)
  else:
    raise NotImplementedError

  # do the matching
  best_match_name, best_match_cost = matcher.match(input_xyz)
  print 'Best match name:', best_match_name, 'with cost:', best_match_cost
  draw_comparison(left_cloud=input_xyz, right_cloud=dataset[best_match_name]['cloud_xyz'])

if __name__ == '__main__':
  main()
