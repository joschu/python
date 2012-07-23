"""
Simple method for segmentation of organized point cloud
"""


from __future__ import division
import numpy as np
import rospy
from brett2 import ros_utils
from brett2.ros_utils import RvizWrapper,Marker
from utils.math_utils import norms
import cv2
import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
import scipy.ndimage as ndi
import math

HEIGHT_HIST_RES = .0005
ABOVE_TABLE_CUTOFF = .01
OBJECT_CLUSTER_TOL=.05
MIN_CLUSTER_SIZE = 100

def find_peak(x,y):
    imax = y.argmax()
    ilo = max(imax-5,0)
    ihi = min(imax+6, len(x))
    a,b,c = np.polyfit(x[ilo:ihi],y[ilo:ihi],2)
    out =  - b/(2*a)
    
    if out < x[ilo] or out > x[ihi]:
        out = x[imax]
    
    #import matplotlib.pyplot as plt
    #plt.figure(123)
    #plt.plot(x,y)
    #plt.vlines([out],0, 1000,colors='r')
    #plt.vlines([x[imax]],0,1000, colors='g')
    #plt.show()
    return out
def get_table_height(xyz):
    z = xyz[:,:,2]
    zvals = z[np.isfinite(z)]
    zmin = zvals.min()
    zmax = zvals.max()
    bins = np.arange(zmin, zmax, HEIGHT_HIST_RES)
    counts,_ = np.histogram(zvals, bins=bins)
    table_height = find_peak((bins[1:]+bins[:-1])/2, counts)
    return table_height

def get_table_dimensions(xyz):
    table_height = get_table_height(xyz)
    z = xyz[:,:,2]
    above_table_mask = np.isfinite(z) & (z > table_height + ABOVE_TABLE_CUTOFF)
    table_pts = xyz[(z > table_height-.01) & (z < table_height + .01)]
    xmin = table_pts[:,0].min()
    xmax = table_pts[:,0].max()
    ymin = table_pts[:,1].min()
    ymax = table_pts[:,1].max()
    zmax = table_height
    zmin = table_height-.02
    
    return xmin, xmax, ymin, ymax, zmin, zmax
            
def get_cylinder(xyz):
    center,radius = cv2.minEnclosingCircle(xyz[:,:2].reshape(-1,1,2).astype('float32')*100000)
    x,y = center
    x/=100000
    y/=100000
    radius/=100000
    zmin = xyz[:,2].min()
    zmax = xyz[:,2].max()
    return (x,y,(zmax+zmin)/2,radius, zmax-zmin)
    

marker_handles = []
            
FIG_DEPTH = 9
FIG_FP_LABELS = 10
FIG_LABELS = 11
FIG_ABOVE = 12
FIG_BGR = 13
            
def segment_tabletop_scene(xyz, frame_id='base_footprint', plotting2d = False, plotting3d = False, plotting2d_all = False):
    """
    
    Initial segmentation:
    1. Median filter point cloud to remove NaNs
    2. Select the points that lie above the table
    3. Select the points that don't lie above or left of a discontinuity in x, y, or z
    4. Find connected components
    
    
    Second pass:
    5. Fit cylinders to each cluster
    6. Merge overlapping cylinders
    
    In the future, fit ellipses or boxes in addition to cylinders, 
    since some objects are oblong.
    
    """
    
    assert xyz.ndim == 3 and xyz.shape[0] > 1 and xyz.shape[1] > 1

    np.putmask(xyz, np.isnan(xyz), -np.inf)
    xyz_mf = ndi.median_filter(xyz, size=(3,3,1))

    rgrad = xyz_mf[1:,:-1,:] - xyz_mf[:-1,:-1,:]
    cgrad = xyz_mf[:-1,1:,:] - xyz_mf[:-1,:-1,:]    
    
    rdiff = (rgrad**2).sum(axis=2)
    cdiff = (cgrad**2).sum(axis=2)
    
    small_diff_mask_interior = (rdiff < OBJECT_CLUSTER_TOL**2) & (cdiff < OBJECT_CLUSTER_TOL**2)
    small_diff_mask = np.zeros(xyz.shape[:2],bool)
    small_diff_mask[:-1, :-1] = small_diff_mask_interior
        

    table_height = get_table_height(xyz)
    z = xyz[:,:,2]
    above_table_mask = np.isfinite(z) & (z > table_height + ABOVE_TABLE_CUTOFF)

    
    both_mask = small_diff_mask & above_table_mask

    labels, max_label = ndi.label(both_mask)
    label_counts = np.bincount(labels.flatten())
        
    n_clu = (label_counts[1:] >= MIN_CLUSTER_SIZE).sum()
    old2new = np.arange(max_label+1)
    old2new[label_counts < MIN_CLUSTER_SIZE] = 0
    old2new[1:][label_counts[1:] >= MIN_CLUSTER_SIZE] = np.arange(n_clu)+1
    labels = old2new[labels]
    
    clusters = [xyz[labels == i] for i in xrange(1, n_clu+1)]
    merge_to = np.arange(n_clu)
    centers_radii = [cv2.minEnclosingCircle(clu[:,:2].reshape(-1,1,2).astype('float32')*100000) 
                     for clu in clusters]
    for i in xrange(n_clu):
        for j in xrange(i+1, n_clu):
            (x0, y0), r0 = centers_radii[i]
            (x1, y1), r1 = centers_radii[j]
            dist = math.hypot(x1-x0, y1-y0)
            rad_sum = r0 + r1
            if dist < rad_sum:
                # i,j,dist,rad_sum
                merge_to[merge_to==merge_to[j]] = merge_to[i]
    
    final_clusters = [np.concatenate([clusters[i] for i in np.flatnonzero(merge_to==m)],0)
                      for m in np.flatnonzero(np.bincount(merge_to))]
    
    if plotting2d:
        import matplotlib.pyplot as plt
        plt.close('all')
        
        plt.figure(FIG_LABELS)
        labels_merged = merge_to[labels-1]+1
        labels_merged[labels==0] = 0
        plt.imshow(labels_merged)
        plt.title("after merging")        
        for (i,cluster) in enumerate(final_clusters):
            row,col = np.unravel_index(norms(xyz - cluster.mean(axis=0)[None,None,:],2).argmin(),xyz.shape[0:2])
            # print "center pixel:",i,row,col
            plt.text(col,row,str(i))

        
    if plotting2d_all:

        plt.figure(FIG_ABOVE)
        plt.imshow(both_mask)
        plt.title("above table & continuity")

        plt.figure(FIG_FP_LABELS)
        plt.imshow(labels)
        plt.title("first-pass labels")        
        
        plt.figure(FIG_DEPTH)
        plt.imshow(z)        

    if plotting2d: 
        plt.show()
            


        
    if plotting3d:
        global marker_handles
        marker_handles = []
        
        for clu in final_clusters:
            x,y,z,r,h = get_cylinder(clu)
            rviz = RvizWrapper.create()
            ps = gm.PoseStamped()
            ps.pose.position = gm.Point(x,y,z)
            ps.pose.orientation = gm.Quaternion(0,0,0,1)
            ps.header.frame_id = frame_id
            marker_handles.append(rviz.draw_marker(ps, type=Marker.CYLINDER, scale=(2*r, 2*r, h), rgba=(1,1,0,.2), ns = "segment_tabletop_scene"))


        
    return final_clusters
           
