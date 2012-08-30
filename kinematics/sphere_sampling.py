from __future__ import division
import numpy as np
from numpy import r_
from jds_utils import math_utils
import itertools
import scipy.spatial.distance as ssd

def get_sphere_points(n_subdivisions):
    phi = (1+np.sqrt(5))/2
    verts = []
    verts.extend(r_[0,y,z] for y in [-1,1] for z in [-phi,phi])
    verts.extend(r_[x,y,0] for x in [-1,1] for y in [-phi,phi])
    verts.extend(r_[x,0,z] for x in [-phi,phi] for z in [-1,1])
    
    dists = ssd.cdist(verts, verts)
    triangles = []
    edge_length = dists[dists>0].min()
    for (i,j,k) in itertools.combinations(xrange(len(verts)), 3):
        if dists[i,j] == edge_length and dists[j,k] == edge_length and dists[k,i] == edge_length:
            triangles.append((i,j,k))

        
    faces = [(verts[i], verts[j], verts[k]) for (i,j,k) in triangles]

    for _i_sub in xrange(n_subdivisions):
        new_faces = []
        for (v0, v1, v2) in faces:
            new_faces.extend([
            (v0, (v0 + v1)/2, (v0 + v2)/2),
            (v1, (v1 + v0)/2, (v1 + v2)/2),
            (v2, (v2 + v0)/2, (v2 + v1)/2),            
            ((v0 + v1)/2, (v1 + v2)/2, (v2 + v0)/2)])
        faces = new_faces
        
    all_verts = np.array(faces).reshape(-1,3)
    all_verts = remove_duplicate_rows(all_verts[np.lexsort(all_verts.T)])
    all_verts = math_utils.normr(all_verts)
    return all_verts
    
        
        
def remove_duplicate_rows(mat):
    diffs = mat[1:] - mat[:-1]
    return mat[np.r_[True,(abs(diffs) >= 1e-6).any(axis=1)]]