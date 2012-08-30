#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("infile",type=argparse.FileType('r'))
args = parser.parse_args()

import cPickle, os
import scikits.learn as skl, numpy as np
image_classifier = cPickle.load(args.infile)
classifier = image_classifier.classifier
coeffs = classifier.coef_
intercepts = classifier.intercept_


coeffs_file = "coeffs.txt"
intercepts_file = "intercepts.txt"
print "writing",coeffs_file,"and",intercepts_file
np.savetxt(coeffs_file,coeffs)
np.savetxt(intercepts_file,intercepts)