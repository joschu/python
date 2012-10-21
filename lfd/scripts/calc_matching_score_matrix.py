#!/usr/bin/env python

import numpy as np
import lfd
from lfd import recognition
import yaml, os
import os.path as osp

def read_training_data(spec_file):

  spec = None
  with open(spec_file, 'r') as specf:
    spec = yaml.load(specf)

  dirname = osp.dirname(spec_file)
  classes = spec['classes']

  data = {}
  for seg in classes:
    segdata = []
    for ex in classes[seg]['examples']:
      filename = osp.join(dirname, ex)
      with open(filename, 'r') as exf:
        npf = np.load(exf)
        segdata.append({'cloud_xyz': npf['cloud_xyz'], 'cloud_xyz_ds': npf['cloud_xyz_ds']})
    data[seg] = segdata

  return data

def make_matcher(method, dataset):
  if method == 'geodesic_dist':
    matcher = recognition.GeodesicDistMatcher(dataset)
  elif method == 'shape_context':
    matcher = recognition.ShapeContextMatcher(dataset)
  else:
    raise NotImplementedError
  return matcher

def main():
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--spec', help='training spec yaml file')
  #parser.add_argument('--dataset', default='overhand_knot', help='name of dataset')
  parser.add_argument('--method', choices=['geodesic_dist', 'shape_context', 'geodesic_dist+shape_context'], default='geodesic_dist', help='matching algorithm')
  parser.add_argument('--out_prefix', default=None)
  args = parser.parse_args()

  if args.out_prefix is None:
    args.out_prefix = osp.join(osp.dirname(args.spec), args.method)
    print 'Will write with prefix', args.out_prefix

  #dataset = recognition.DataSet.LoadFromTaskDemos(args.dataset)

# elif args.method == 'geodesic_dist+shape_context':
#   matcher = recognition.CombinedNNMatcher(dataset, [recognition.GeodesicDistMatcher, recognition.ShapeContextMatcher], [1, 0.1])

  # read training data
  training = read_training_data(args.spec)
  print 'Read', len(training), 'segments.'

  def itertraining(tr):
    for seg in sorted(tr.keys()):
      for ex in tr[seg]:
        yield ex
 
  # compute cost between all pairs
  mat = []
  for i, ex1 in enumerate(itertraining(training)):
    row = []
    for j, ex2 in enumerate(itertraining(training)):
      singleton = recognition.DataSet.LoadFromDict({'ex1': ex1})
      matcher = make_matcher(args.method, dataset=singleton)
      _, cost = matcher.match(ex2['cloud_xyz'])
      row.append(cost)
      print '(%d, %d): %f' % (i, j, cost)
    mat.append(row)

  # output csv and npz

  csv = '\n'.join(','.join(map(str, l)) for l in mat)
  print '===== CSV ========================='
  print csv
  with open(args.out_prefix + '.csv', 'w') as f:
    print >>f, csv
  print
  print 'Wrote', args.out_prefix + '.csv'

  np.savez(args.out_prefix, mat=np.array(mat))
  print 'Wrote', args.out_prefix + '.npz'

if __name__ == '__main__':
  main()
