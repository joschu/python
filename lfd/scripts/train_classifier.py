#!/usr/bin/env python

import numpy as np
import yaml, os
import os.path as osp

def main():
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--spec', help='training spec yaml file')
  parser.add_argument('--viz', action='store_true', help='just visualize')
  parser.add_argument('--no_normalize', action='store_false', help='do not normalize data by stdev')
  args = parser.parse_args()

  # load spec and feature value matrix
  spec = None
  with open(args.spec, 'r') as specf:
    spec = yaml.load(specf)
  feature_data = {}
  for feature_name in spec['features']:
    filename = osp.join(osp.dirname(args.spec), spec['features'][feature_name]['data'])
    with open(filename, 'r') as feature_data_file:
      feature_data[feature_name] = np.load(feature_data_file)['mat']


  feature_names = sorted(feature_data.keys())

  # create data points with labels
  LABEL_SAME = 1; LABEL_DIFFERENT = -1
  # data has rows [feature_0 feature_1 label]
  npts = feature_data.values()[0].size
  data = np.zeros((npts, len(feature_names)+1))
  for fnum, feature_name in enumerate(feature_data):
    data[:,fnum] = feature_data[feature_name].reshape((-1, 1))[:,0]
  labels = np.empty_like(feature_data.values()[0])
  labels[:,:] = LABEL_DIFFERENT
  start = 0
  for i, cls in enumerate(sorted(spec['classes'].keys())):
    num_in_cls = len(spec['classes'][cls]['examples'])
    labels[start:start+num_in_cls, start:start+num_in_cls] = LABEL_SAME
    start += num_in_cls
  data[:,-1] = labels.reshape((-1, 1))[:,0]

  if args.viz:
    import pylab as pl
    pl.scatter(data[:,0], data[:,1], s=30, c=data[:,2], cmap=pl.cm.Paired)
    pl.xlabel(feature_data.keys()[0])
    pl.ylabel(feature_data.keys()[1])
    pl.show()

  from sklearn import neighbors, datasets, linear_model, svm, pipeline, preprocessing

  classifiers = dict(
    knn=neighbors.KNeighborsClassifier(),
    logistic=linear_model.LogisticRegression(C=1e5),
    svm=svm.SVC(C=1e5, kernel='linear'),
  )

  X = data[:,0:2]
  Y = data[:,2]

  trained = {}
  for name, clf in classifiers.iteritems():
    pclf = pipeline.Pipeline([
      ('scaler', preprocessing.Scaler()),
      ('classifier', clf)
    ])
    pclf.fit(X, Y)
    trained[name] = pclf

  filename = osp.join(osp.dirname(args.spec), 'classifiers.pkl')
  print 'Writing classifiers to', filename
  with open(filename, 'wb') as f:
    import cPickle
    cPickle.dump(trained, f)

  if args.viz:
    h = .01  # step size in the mesh
    import pylab as pl
    fignum = 1
    for name, pclf in trained.iteritems():
      # Plot the decision boundary. For that, we will asign a color to each
      # point in the mesh [x_min, m_max]x[y_min, y_max].
      x_min, x_max = X[:, 0].min(), X[:, 0].max()
      y_min, y_max = X[:, 1].min(), X[:, 1].max()
      xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
              np.arange(y_min, y_max, h))
      Z = pclf.predict(np.c_[xx.ravel(), yy.ravel()])

      # Put the result into a color plot
      Z = Z.reshape(xx.shape)
      #pl.figure(fignum, figsize=(6, 6))
      pl.figure(fignum)
      pl.pcolormesh(xx, yy, Z, cmap=pl.cm.Paired)

      # Plot also the training points
      pl.scatter(X[:, 0], X[:, 1], c=Y, cmap=pl.cm.Paired)
      pl.title(name)
      pl.xlabel(feature_data.keys()[0])
      pl.ylabel(feature_data.keys()[1])
      fignum += 1
    pl.show() 


if __name__ == '__main__':
  main()
