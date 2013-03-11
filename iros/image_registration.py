import numpy as np
import cv2
import scipy.ndimage as ndi
import itertools as it
import scipy.spatial.distance as ssd
from jds_utils.shortest_paths import shortest_paths

DEBUG_PLOTTING = False
PLOT_COUNTER = 0
class Colors:
    RED = (0,0,255)
    GREEN = (0,255,0)
    BLUE = (255,0,0)


def register_images(src_img, targ_img, src_xys_n):
    xys_n = []
    vs_n = []
    rankvs_n = []
    for src_xy in src_xys_n:
        targ_xys, vs = get_matches(src_img, src_xy, targ_img)
        xys_n.append(targ_xys)
        vs_n.append(vs)
        rankvs_n.append(0*np.arange(len(vs)))
    
    matchinds_n = dp_match(xys_n, rankvs_n, src_xys_n)
    return [xys_n[i][m] for (i,m) in enumerate(matchinds_n)]
    

def dp_match(xys_n, vs_n, src_xys_n):
    N = len(xys_n)
    assert len(vs_n) == N
    assert len(src_xys_n) == N
           
    nodecosts = vs_n
    edgecosts = []
    
    for n in xrange(N-1):
        xys_ad = xys_n[n]
        xys_bd = xys_n[n+1]
        A = len(xys_ad)
        B = len(xys_bd)
        D = len(xys_ad[0])
        diff_d = (np.array(src_xys_n[n]) - np.array(src_xys_n[n+1]))[None,:]
        diff_abd = xys_ad[:,None,:] - xys_bd[None,:,:]
        diffdiff_abd = diff_abd - diff_d
        edgecosts_ab = np.sqrt((diffdiff_abd**2).sum(axis=2))
        edgecosts.append(edgecosts_ab)

    paths_mn, costs_m = shortest_paths(nodecosts, edgecosts)
    bestpath_n = paths_mn[np.argmin(costs_m)]
    return bestpath_n



def get_matches(src_img, xy, targ_img, patch_size = 59, max_deficit = .3, min_match_distance = 10, max_num_matches = 20):
    assert patch_size % 2 == 1
    patch_radius = patch_size // 2
    assert src_img.ndim == 3
    col_keypoint, row_keypoint = xy

    assert row_keypoint - patch_radius >= 0
    assert row_keypoint + patch_radius < src_img.shape[0]
    assert col_keypoint - patch_radius >= 0
    assert col_keypoint + patch_radius < src_img.shape[1]
    
    
    patch = src_img[row_keypoint-patch_radius:row_keypoint+patch_radius+1, 
                      col_keypoint-patch_radius:col_keypoint+patch_radius+1, :]
    
    heatmap = cv2.matchTemplate(targ_img, patch, cv2.TM_CCOEFF_NORMED)
    
    sorted_heatmap_values = np.sort(heatmap.flatten())
    abs_thresh = sorted_heatmap_values[-1] - max_deficit
    
    heatmap_maxes = ndi.maximum_filter(heatmap, size=(min_match_distance, min_match_distance))
    rows_local_maxima, cols_local_maxima = np.nonzero(heatmap_maxes == heatmap)
    
    
    rc_good_local_maxima = []
    vals_good_local_maxima = []

    for (row_lm, col_lm) in zip(rows_local_maxima, cols_local_maxima):
        val = heatmap[row_lm, col_lm]
        if val >= abs_thresh:
            rc_good_local_maxima.append((row_lm, col_lm))
            vals_good_local_maxima.append(val)
            
    rc_good_local_maxima = np.array(rc_good_local_maxima)
    vals_good_local_maxima = np.array(vals_good_local_maxima)
    sort_inds = (-vals_good_local_maxima).argsort()[:max_num_matches]
    
    rc_good_local_maxima = rc_good_local_maxima[sort_inds]
    vals_good_local_maxima = vals_good_local_maxima[sort_inds]
    

    if DEBUG_PLOTTING:
        plot_img = cv2.cvtColor(heatmap, cv2.COLOR_GRAY2BGR)
        for (i,(row, col), v) in zip(it.count(), rc_good_local_maxima, vals_good_local_maxima):
            cv2.putText(plot_img, "%i:%.2f"%(i,v), (col, row), cv2.FONT_HERSHEY_PLAIN, 1.0, Colors.RED, thickness = 1)
            cv2.circle(plot_img, (col, row), 3, Colors.GREEN)
        global PLOT_COUNTER
        cv2.imshow("get_matches_heatmap%i"%PLOT_COUNTER, plot_img)
        PLOT_COUNTER += 1
        while cv2.waitKey(10) == -1: pass
        
    return rc_good_local_maxima[:,::-1] + patch_radius, vals_good_local_maxima

