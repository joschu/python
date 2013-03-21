import numpy as np
import cv2
import scipy.ndimage as ndi
import itertools as it
import scipy.spatial.distance as ssd
import jds_utils.shortest_paths as sp
import jds_utils.math_utils as mu

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
    
    #matchinds_n = dp_match(xys_n, rankvs_n, src_xys_n)
    matchinds_n = dp_match_ternary(xys_n, rankvs_n, src_xys_n)
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

    paths_mn, costs_m = sp.shortest_paths(nodecosts, edgecosts)
    
    
    
    bestpath_n = paths_mn[np.argmin(costs_m)]
    return bestpath_n

def dp_match_ternary(xys_n, vs_n, src_xys_n):
    N = len(xys_n)
    assert len(vs_n) == N
    assert len(src_xys_n) == N
           
    nodecosts = vs_n
    edgecosts = []
    tripcosts = []
    

    diff_d = (np.array(src_xys_n[0]) - np.array(src_xys_n[1]))[None,:]
    diff_abd = xys_n[0][:,None,:] - xys_n[1][None,:,:]
    lengths_ab = mu.norms(diff_abd, 2)
    edge_coef = 10
    edgecosts.append(np.abs(np.log(lengths_ab / np.linalg.norm(diff_d)))*edge_coef)
    
    #edgecosts.append(np.zeros((xys_n[0].shape[0],xys_n[1].shape[0])))
    for n in xrange(2,N):
        xys_ad = xys_n[n-2]
        xys_bd = xys_n[n-1]        
        xys_cd = xys_n[n]
        A = len(xys_ad)
        B = len(xys_bd)
        C = len(xys_cd)
        tripcosts_abc = np.empty((A,B,C))
        oldtri = np.array([src_xys_n[n-2], src_xys_n[n-1], src_xys_n[n]])
        oldsides = np.array([oldtri[1]-oldtri[2], oldtri[2]-oldtri[0], oldtri[0]-oldtri[1]])
        oldcomplex = oldsides[:,0] + 1j*oldsides[:,1]
        oldcomplexratios = np.array([oldcomplex[1]/oldcomplex[2],oldcomplex[2]/oldcomplex[0],oldcomplex[0]/oldcomplex[1]])
        oldsidelengths = mu.norms(oldsides, 1)
        oldangles = np.arccos([oldsides[1].dot(oldsides[2])/np.linalg.norm(oldsides[1])/np.linalg.norm(oldsides[2]), 
                               oldsides[2].dot(oldsides[0])/np.linalg.norm(oldsides[2])/np.linalg.norm(oldsides[0]), 
                               oldsides[0].dot(oldsides[1])/np.linalg.norm(oldsides[0])/np.linalg.norm(oldsides[1])])
        for a in xrange(A):
            for b in xrange(B):
                for c in xrange(C):
                    
                    newtri = np.array([xys_ad[a], xys_bd[b], xys_cd[c]])
                    newsides = np.array([newtri[1]-newtri[2],newtri[2]-newtri[0], newtri[0]-newtri[1]])
                    #newsidelengths = mu.norms(newsides,1)
                    newcomplex = newsides[:,0] + 1j*newsides[:,1]
                    newcomplexratios = np.array([newcomplex[1]/newcomplex[2],newcomplex[2]/newcomplex[0],newcomplex[0]/newcomplex[1]])
                    
                    #newangles = np.arccos([newsides[1].dot(newsides[2])/np.linalg.norm(newsides[1])/np.linalg.norm(newsides[2]), 
                                           #newsides[2].dot(newsides[0])/np.linalg.norm(newsides[2])/np.linalg.norm(newsides[0]), 
                                           #newsides[0].dot(newsides[1])/np.linalg.norm(newsides[0])/np.linalg.norm(newsides[1])])
                    #tripcosts_abc[a,b,c] = np.linalg.norm(newsidelengths - oldsidelengths)
                    #tripcosts_abc[a,b,c] = np.linalg.norm(newangles - oldangles)*10
                    tripcosts_abc[a,b,c] = (np.abs(newcomplexratios - oldcomplexratios)**2).sum()
        tripcosts.append(tripcosts_abc)


        diff_d = (np.array(src_xys_n[n-1]) - np.array(src_xys_n[n]))[None,:]
        diff_bcd = xys_bd[:,None,:] - xys_cd[None,:,:]
        lengths_bc = mu.norms(diff_bcd, 2)
        edgecosts.append(np.abs(np.log(lengths_bc / np.linalg.norm(diff_d)))*edge_coef)

                
        #diff_d = (np.array(src_xys_n[n]) - np.array(src_xys_n[n+1]))[None,:]
        #diff_abd = xys_ad[:,None,:] - xys_bd[None,:,:]
        #diffdiff_abd = diff_abd - diff_d
        #edgecosts_ab = np.sqrt((diffdiff_abd**2).sum(axis=2))
        #edgecosts.append(edgecosts_ab)
        

    bestpath,cost = sp.shortest_paths_ternary(nodecosts, edgecosts,tripcosts)
    print "cost",cost
    return bestpath


def get_matches(src_img, xy, targ_img, patch_size = 25, max_deficit = .3, min_match_distance = 10, max_num_matches = 20):
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

