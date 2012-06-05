import numpy as np
import random

def ransac(data, modelClass, minDataPts, nIter, threshold, nCloseRequired):
    # from http://en.wikipedia.org/wiki/RANSAC
    best_model = None
    best_consensus_set = None
    best_error = np.inf
    for iteration in xrange(nIter):
        maybe_inliers = data[random.sample(range(len(data)),minDataPts)]
        maybe_model = modelClass()
        maybe_model.fit(maybe_inliers)

        fit_errors = maybe_model.calc_error(data)
        consensus_set = np.flatnonzero(fit_errors < threshold)
        
        if len(consensus_set) > nCloseRequired:
            maybe_model.fit(data[consensus_set])
            new_fit_errors = maybe_model.calc_error(data)
            total_error = new_fit_errors.sum()
            if total_error < best_error:
                best_model = maybe_model
                best_consensus_set = np.flatnonzero(new_fit_errors < threshold) # here I differ from wikipedia
                best_error = total_error

    return best_model, best_consensus_set, best_error



class Model(object):    
    def fit(self, data):
        raise
    def calc_error(self, data):
        raise

class ConstantModel(Model):
    def fit(self, data):
        self.mean = np.mean(np.atleast_2d(data),axis=0)
    def calc_error(self, data):
        # distance
        return np.sqrt(((np.atleast_2d(data) - self.mean[None,:])**2).sum(axis=1))
