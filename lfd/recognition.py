"""
compare point clouds
"""


from __future__ import division
import numpy as np
import networkx as nx
from matplotlib.delaunay import Triangulation
import lfd
from lfd import registration, warping
from scipy import sparse
import jds_utils.math_utils as mu



def make_sampling_matrix(inds_list, n_orig):
    """
    inds_list tells you which input inds correspond to each output ind
    "sampling matrix" is n_input_inds x n_output_inds such that
    out_{ij} = Indicator(input i -> output j)  * 1/n_input_inds
    i.e. it tells you how output is weighted avg of input
    """
    row_inds = []
    col_inds = []
    vals = []
    for (i_col, inds) in enumerate(inds_list):
        col_inds.extend([i_col]*len(inds))
        row_inds.extend(inds)
        vals.extend([1/len(inds)]*len(inds))
    return sparse.csc_matrix((vals,np.array([row_inds, col_inds])), shape=(n_orig,len(inds_list)))
            
def calc_geodesic_distances_downsampled_old(xyz, xyz_ds, ds_inds):
    """
    calculate geodesic distances between point xyz_ds using xyz to make graph
    """
    assert xyz.shape[1] == 3
    assert xyz_ds.shape[1] == 3
    D = calc_geodesic_distances(xyz)
    
    S = make_sampling_matrix(ds_inds, len(xyz))
    print S.shape
    return S.transpose().dot(S.transpose().dot(D).transpose()).T


def calc_geodesic_distances_downsampled(xyz, xyz_ds, ds_inds):
    """
    calculate geodesic distances between point xyz_ds using xyz to make graph
    """
    assert xyz.shape[1] == 3
    assert xyz_ds.shape[1] == 3
    D = calc_geodesic_distances(np.concatenate([xyz_ds,xyz],0))
    
    return D[:len(xyz_ds), :len(xyz_ds)]

    
def calc_geodesic_distances(xyz, res=.03):
    """
    Calculates pairwise geodesic distances.
    Note that we generate the graph by projecting to 2D
    """
    x,y = xyz[:,:2].T
    tri = Triangulation(x,y)
    G = nx.Graph()
    #G.add_nodes_from(xrange(len(xyz)))
    for i0 in xrange(len(xyz)):
        G.add_node(i0)
    for (i0, i1) in tri.edge_db:
        dist = np.linalg.norm(xyz[i1] - xyz[i0])
        if dist < res:
            G.add_edge(i0, i1, weight = np.linalg.norm(xyz[i1] - xyz[i0]))
    distmat = np.asarray(nx.floyd_warshall_numpy(G))
    
    finitevals = distmat[np.isfinite(distmat)]
    distmat[~np.isfinite(distmat)] = finitevals.max() * 3
    return distmat


def calc_match_score(xyz0, xyz1, dists0 = None, dists1 = None, plotting = False):
    """
    calculate similarity between xyz0 and xyz1 using geodesic distances
    """
    
    f,info = registration.tps_rpm(xyz0, xyz1, plotting=plotting,reg_init=1,reg_final=.1,n_iter=21, verbose=False, return_full=True)
    partners = info["corr_nm"].argmax(axis=1)
    starts, ends = np.meshgrid(partners, partners)

    reses = [.05, .07, .09]
    nres = len(reses)

    targ_dist_mats, src_dist_mats = [],[]
    
    for i in xrange(nres):
    
        dists0 = calc_geodesic_distances(xyz0, reses[i])
        dists1 = calc_geodesic_distances(xyz1, reses[i])


        dists_targ = dists1[starts, ends]
        
        dists0_normed = dists0 / np.median(dists0)
        dists_targ_normed = dists_targ / np.median(dists1)
        
        src_dist_mats.append(dists0_normed)
        targ_dist_mats.append(dists_targ_normed)
    distmat = np.empty((nres, nres))
    for i in xrange(nres):
        for j in xrange(nres):
            distmat[i,j] = np.abs(src_dist_mats[i] - targ_dist_mats[j]).mean()
            

    print "dist at res:", distmat, distmat.min()
    return distmat.min()

def match_and_calc_shape_context(xyz_demo_ds, xyz_new_ds, hists_demo=None, hists_new=None, normalize_costs=False, return_tuple=False):
  def compare_hists(h1, h2):
    assert h2.shape == h1.shape
    # (single terms of) chi-squared test statistic
    return 0.5/h1.shape[0] * ((h2 - h1)**2 / (h2 + h1 + 1))

  f = registration.tps_rpm(xyz_demo_ds, xyz_new_ds, plotting=False,reg_init=1,reg_final=.1,n_iter=21, verbose=False)
  partners = f.corr.argmax(axis=1)

  if hists_demo is None:
    hists_demo = calc_shape_context_hists(xyz_demo_ds)
  if hists_new is None:
    hists_new = calc_shape_context_hists(xyz_new_ds[partners])

  costs = compare_hists(hists_demo, hists_new).sum(axis=1).sum(axis=1)
  if normalize_costs:
    print max(costs)
    m = max(0.5, max(costs))
    if m != 0: costs /= m

  if return_tuple:
    return f, partners, hists_demo, hists_new, costs

  return costs

def calc_shape_context_hists(xy, logr_bins=10, theta_bins=8, normalize_angles=False, pca_radius=0.1):
#def calc_shape_context_hists(xy, logr_bins=10, theta_bins=20, normalize_angles=False, pca_radius=0.1):
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

    return np.histogram2d( \
      lp[:,0], lp[:,1], \
      bins=(logr_bins, theta_bins), \
      range=((-1e6, 3), (0, 2.*np.pi)) \
    )[0]

  hists = np.zeros((npoints, logr_bins, theta_bins))
  for k in xrange(npoints):
    hists[k] = calc_one_hist(xy[k])
#    hists[k] = (hists[k] > 0).astype('float')
  return hists

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
    import h5py, yaml, os
    import os.path as osp
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

  @staticmethod
  def LoadFromDict(d):
    ds = DataSet()
    ds.data = d
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
      if 'geodesic_dists' not in demo:
        demo["geodesic_dists"] = calc_geodesic_distances_downsampled_old(demo["cloud_xyz"], demo["cloud_xyz_ds"], ds_inds)

  def get_name(self):
    return 'GeodesicDistMatcher'

  def preprocess_input(self, xyz_new):
    xyz_new_ds, ds_inds = downsample(xyz_new)
    dists_new = calc_geodesic_distances_downsampled_old(xyz_new, xyz_new_ds, ds_inds)
    return xyz_new_ds, dists_new

  def calc_cost(self, dataset_item, preprocessed_input):
    xyz_new_ds, dists_new = preprocessed_input
    xyz_demo_ds = np.squeeze(dataset_item["cloud_xyz_ds"])
    dists_demo = dataset_item['geodesic_dists']
    cost = calc_match_score(xyz_demo_ds, xyz_new_ds, dists0=dists_demo, dists1=dists_new)
    return cost


class ShapeContextMatcher(NearestNeighborMatcher):
  def __init__(self, dataset):
    BasicMatcher.__init__(self, dataset)
    for _, demo in self.dataset.items():
      demo["cloud_xyz_ds"], ds_inds = downsample(demo["cloud_xyz"])
      demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
      if 'shape_context_hists' not in demo:
        demo['shape_context_hists'] = calc_shape_context_hists(demo['cloud_xyz_ds'])

  def get_name(self):
    return 'ShapeContextMatcher'

  def preprocess_input(self, xyz_new):
    xyz_new_ds, ds_inds = downsample(xyz_new)
    hists_new = calc_shape_context_hists(xyz_new_ds)
    return xyz_new_ds, hists_new

  def calc_cost(self, dataset_item, preprocessed_input):
    xyz_new_ds, hists_new = preprocessed_input
    xyz_demo_ds, hists_demo = np.squeeze(dataset_item["cloud_xyz_ds"]), dataset_item['shape_context_hists']
    costs = match_and_calc_shape_context(xyz_demo_ds, xyz_new_ds)#, hists_demo, hists_new)
    dataset_item['shape_context_costs'] = costs
    return costs.sum()
