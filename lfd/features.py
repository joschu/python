from __future__ import division
from utils import math_utils
import numpy as np

def cloud_shape_context(cloud, hist_dim):
    return np.array([point_shape_context(pt, cloud, hist_dim) for pt in cloud])

def point_shape_context(pt, cloud, hist_dim):
    cloud = np.atleast_2d(cloud) - np.atleast_2d(pt)

    r = math_utils.norms(cloud,1)
    r_med = np.median(r)
    
    if hist_dim == 2:
        
        num_r_bins = 4
        num_theta_bins = 9
        
        r_bin_edges = r_med/2**(num_r_bins-1) * 2**np.arange(0, num_r_bins+1)
        theta = np.arctan2(cloud[:,1], cloud[:,0])
        theta_bin_edges = np.linspace(-np.pi, np.pi, num_theta_bins+1, endpoint = True)

        hist,_,_ = np.histogram2d(r,theta, bins = (r_bin_edges, theta_bin_edges))
        
        areas = np.pi * (r_bin_edges[1:]**2 - r_bin_edges[:-1]**2)/num_theta_bins
        
        features = ((areas**(-1/2))[:,None] * hist).flatten()
        features /= features.sum()
        
        return features
        
        
    else:
        raise NotImplementedError
    
    