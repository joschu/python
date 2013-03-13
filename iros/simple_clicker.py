
import cv2
import numpy as np
import sys

from jds_utils.colorize import colorize


def show_pointclouds(clouds, window_name):
    nc = len(clouds)

    if nc == 2:
        rgb_plot = clouds[1].copy()
        cv2.imshow(window_name, rgb_plot)
        cv2.waitKey(100)
    else:
        rgb_plot_multi = []
        for i in range(nc):
            rgb_plot_multi.append(clouds[i][1].copy())
            cv2.imshow(window_name, rgb_plot_multi[i])
            cv2.waitKey(100)

#########################################
### find a single keypoint
#########################################
def find_kp(kp, xyz_tf, rgb_plot, window_name):
    ### clicking set-up 
    class GetClick:
        x = None
        y = None
        done = False
        def callback(self, event, x, y, flags, param):
            if self.done:
                return
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.x = x
                self.y = y
                self.done = True

    print colorize("click on the center of the " + kp, 'red', bold=True)
    gc = GetClick()
    cv2.setMouseCallback(window_name, gc.callback)
    while not gc.done:
        cv2.imshow(window_name, rgb_plot)
        cv2.waitKey(10)
    row_kp = gc.x
    col_kp = gc.y
    
    cv2.circle(rgb_plot, (row_kp, col_kp), 5, (0, 0, 255), -1)
    cv2.imshow(window_name, rgb_plot)    
    cv2.waitKey(100)

    xyz_tf[np.isnan(xyz_tf)] = -2
    x, y, z = xyz_tf[col_kp, row_kp]

    print kp, "3d location", x, y, z

    return (x, y, z)


#########################################
### find cut
#########################################
def find_cut(xyz_tf, rgb_plot, window_name):
    
    ### clicking set-up 
    class GetClick:
        x = None
        y = None
        xy = None
        done = False
        def callback(self, event, x, y, flags, param):
            if self.done:
                return
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.x = x
                self.y = y
                self.xy = (x,y)
                self.done = True    
    
    xy_t = []
    xy_m = []
    xy_b = []    
    
    #print colorize("click at the top of the cut line", 'red', bold=True)
    #gc = GetClick()
    #cv2.setMouseCallback(window_name, gc.callback)
    #while not gc.done:
        #cv2.imshow(window_name, rgb_plot)
        #cv2.waitKey(10)
    #xy_t.append(gc.x)
    #xy_t.append(gc.y)
    
    #x_tcut, y_tcut, z_tcut = xyz_tf[xy_t[1], xy_t[0]]
    #tcut_loc = (x_tcut, y_tcut, z_tcut)
    #cv2.circle(rgb_plot, (xy_t[0], xy_t[1]), 5, (0, 0, 255), -1)
    #cv2.imshow(window_name, rgb_plot)
    #cv2.waitKey(100)

    print colorize("click the cut line in between the holes", 'red', bold=True)
    gc = GetClick()
    cv2.setMouseCallback(window_name, gc.callback)
    while not gc.done:
        cv2.imshow(window_name, rgb_plot)
        cv2.waitKey(10)
    xy_m.append(gc.x)
    xy_m.append(gc.y)
    
    x_mcut, y_mcut, z_mcut = xyz_tf[xy_m[1], xy_m[0]]
    mcut_loc = (x_mcut, y_mcut, z_mcut)
    cv2.circle(rgb_plot, (xy_m[0], xy_m[1]), 5, (0,0, 255), -1)    
    cv2.imshow(window_name, rgb_plot)
    cv2.waitKey(100)

    #print colorize("click the bottom of the cut line", 'red', bold=True)
    #gc = GetClick()
    #cv2.setMouseCallback(window_name, gc.callback)
    #while not gc.done:
        #cv2.imshow(window_name, rgb_plot)
        #cv2.waitKey(10)
    #xy_b.append(gc.x)
    #xy_b.append(gc.y)

    #x_bcut, y_bcut, z_bcut = xyz_tf[xy_b[1], xy_b[0]]
    #bcut_loc = (x_bcut, y_bcut, z_bcut) 
      
    #cv2.circle(rgb_plot, (xy_b[0], xy_b[1]), 5, (0, 0, 255), -1)
    #cv2.imshow(window_name, rgb_plot)
    #cv2.waitKey(100)

    #print 'cut top 3d location', x_tcut, y_tcut, z_tcut
    print 'cut middle cut 3d location', x_mcut, y_mcut, z_mcut
    #print 'cut bottom 3d location', x_bcut, y_bcut, z_bcut

    return mcut_loc


#########################################
### find needle
#########################################
def find_needle_tip(xyz_tfs, rgb_plots, window_name, caller):

    ### clicking set-up 
    class GetClick:
        xy = None
        done = False
        def callback(self, event, x, y, flags, param):
            if self.done:
                return
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.xy = (x,y)
                self.done = True         

    needle_rect_corners = []
    nc = len(xyz_tfs)
    rgb_plot = rgb_plots[0].copy()

    print colorize("click at the corners of a rectangle which encompasses the needle tip", 'red', bold=True)

    for i in xrange(2):
        gc = GetClick()
        cv2.setMouseCallback(window_name, gc.callback)
        while not gc.done:
            cv2.imshow(window_name, rgb_plot)
            cv2.waitKey(10)
        needle_rect_corners.append(gc.xy)

    xy_tl = np.array(needle_rect_corners).min(axis=0)
    xy_br = np.array(needle_rect_corners).max(axis=0)

    cv2.rectangle(rgb_plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0))
    cv2.imshow(window_name, rgb_plot)
    cv2.waitKey(10)

    colmin, rowmin = xy_tl
    colmax, rowmax = xy_br

    row_needle = []
    col_needle = []

    xneedle = np.zeros((nc))
    yneedle = np.zeros((nc))
    zneedle = np.zeros((nc))

    # extract depths from drawn rectangle
    for i in range(nc):
 
        z_rectangle = xyz_tfs[i][rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle
        z_rectangle[np.isnan(z_rectangle)] = -2
        row_needle_temp, col_needle_temp = np.unravel_index(z_rectangle.argmax(), z_rectangle.shape)

        col_needle_temp += colmin # since these indices are from small rectangle
        row_needle_temp += rowmin

        col_needle.append(col_needle_temp)
        row_needle.append(row_needle_temp)
        
        xneedle[i], yneedle[i], zneedle[i] = xyz_tfs[i][row_needle_temp, col_needle_temp]

        del z_rectangle

    ind = np.argmax(zneedle)

    max_needle = []
    max_needle.append(xneedle[ind])
    max_needle.append(yneedle[ind])
    max_needle.append(zneedle[ind])

    if caller == 'process':
        cv2.circle(rgb_plots[ind], (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2)
        cv2.imshow(window_name, rgb_plots[ind])
        cv2.waitKey(100)
        
    else:
        cv2.circle(rgb_plot, (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2)
        cv2.imshow(window_name, rgb_plot)
        cv2.waitKey(100)    
       
    print 'needle tip 3d location', max_needle

    # plot the point cloud with a circle around "highest" point    
    #from mayavi import mlab
    #x,y,z = clouds[ind][0][~np.isnan(clouds[ind][0][:,:,0])].T    
    #mlab.points3d(x,y,z,color=(1,0,0), mode="2dvertex")
    #mlab.points3d([max_needle[0]], [max_needle[1]], [max_needle[2]], color=(0,1,0), mode="sphere", scale_factor=.04, opacity=.2)
    #mlab.show()

    #raw_input("press enter when done looking")        
    #print "at end of needle tip func"

    return max_needle

#########################################
### find needle
#########################################
def find_needle_end(xyz_tfs, rgbs, window_name):

    ### clicking set-up 
    class GetClick:
        xy = None
        done = False
        def callback(self, event, x, y, flags, param):
            if self.done:
                return
            elif event == cv2.EVENT_LBUTTONDOWN:
                self.xy = (x,y)
                self.done = True

    needle_rect_corners = []
    nc = len(xyz_tfs)
    rgb_plot = rgbs[0].copy()
    print colorize("click at the corners of a rectangle which encompasses the end of the needle", 'red', bold=True)

    for i in xrange(2):
        gc = GetClick()
        cv2.setMouseCallback(window_name, gc.callback)
        while not gc.done:
            cv2.imshow(window_name, rgb_plot)
            cv2.waitKey(10)
        needle_rect_corners.append(gc.xy)

    xy_tl = np.array(needle_rect_corners).min(axis=0)
    xy_br = np.array(needle_rect_corners).max(axis=0)

    cv2.rectangle(rgb_plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0))
    cv2.imshow(window_name, rgb_plot)
    cv2.waitKey(100)

    colmin, rowmin = xy_tl
    colmax, rowmax = xy_br
    
    high_red_xyz = []
    high_red_rc = []

    valid_pts = 0
    # extract depths from drawn rectangle

    FOAM_HEIGHT = .85
    
    for i in range(nc):        
        z_rectangle = xyz_tfs[i][rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle
        
        hsv = cv2.cvtColor(rgbs[i][rowmin:rowmax, colmin:colmax].copy(),cv2.COLOR_BGR2HSV)
        h=hsv[:,:,0]
        s=hsv[:,:,1]
        v=hsv[:,:,2]
        
        color_mask = ((h<20) | (h>150)) & (s > 100) & (v > 90)
        height_mask = z_rectangle > FOAM_HEIGHT + .025
        
        total_mask = color_mask & height_mask
        print "%ith image has %i valid points (above foam and red)"%(i, total_mask.sum())
        valid_pts += total_mask.sum()
        
        high_red_xyz.extend(xyz_tfs[i][rowmin:rowmax, colmin:colmax, :][total_mask])
    
    xyz_avg = np.median(high_red_xyz,axis=0)
    
    from jds_image_proc.pcl_utils import xyz2uv
    row, col = xyz2uv(xyz_avg).astype('int')[0]      

    cv2.circle(rgb_plot, (row, col), 3, (255, 0, 0), 2)
    cv2.imshow(window_name, rgb_plot)
    cv2.waitKey(100)    

    print 'Needle end location', xyz_avg
 
    return xyz_avg, valid_pts