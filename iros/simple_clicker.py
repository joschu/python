
import cv2
import numpy as np
import sys
import rospy

from jds_utils.colorize import colorize
import brett2.ros_utils as ru
import sensor_msgs.msg as sm


WIN_NAME = "Find Keypoints"

def get_kp_clouds(listener, cloud_topic, num_clouds):
       
    class MessageGetter:
        
        def __init__(self, n_msgs):
            self.n_msgs = n_msgs
            self.msgs = []
            self.done = False
        def callback(self,msg):
            if self.done:
                return
            elif len(self.msgs) < self.n_msgs:
                self.msgs.append(msg)
            else:
                self.done = True
                
    def get_msgs(n_msgs, topic, msgtype):
        print "waiting for %i messages on topic %s"%(n_msgs, topic)
        mg = MessageGetter(n_msgs)
        sub = rospy.Subscriber(topic, msgtype, mg.callback)
        while not mg.done:
            rospy.sleep(.05)
        sub.unregister()
        return mg.msgs
              
    msgs = get_msgs(num_clouds, cloud_topic, sm.PointCloud2)
    
    if num_clouds == 1:
        msg = msgs[0]
        xyz, rgb = ru.pc2xyzrgb(msg)
    
        if (xyz.shape[0] == 1 or xyz.shape[1] == 1): raise Exception("needs to be an organized point cloud")	
    
        xyz_tf = ru.transform_points(xyz, listener, "base_footprint", "/camera_rgb_optical_frame")        
        rgb_plot = rgb.copy() 
        
        return xyz_tf, rgb_plot

    else:    
        xyz_tfs = []
        rgb_plots = []        
        for i in range(num_clouds):
            msg = msgs[i]            
            xyz, rgb = ru.pc2xyzrgb(msg)
           
            if (xyz.shape[0] == 1 or xyz.shape[1] == 1): raise Exception("needs to be an organized point cloud")
    
            xyz_tf = ru.transform_points(xyz, listener, "base_footprint", "/camera_rgb_optical_frame")        
        
            xyz_tfs.append(xyz_tf)
            rgb_plots.append(rgb)
    
        return xyz_tfs, rgb_plots


def get_last_kp_loc(exec_keypts, desired_keypt, current_seg):        
    
    search_seg = current_seg - 1
       
    while(True):
        if search_seg < 0:
            print "Reached beginning of execution and couldn't find desired keypoint! Aborting..."
            sys.exit(1)            
        else:
            search_seg_names = exec_keypts[search_seg]["names"]
            search_seg_locs = exec_keypts[search_seg]["locations"]
        
            for k in range(len(search_seg_names)):
                if search_seg_names[k] == desired_keypt:
                    kp_loc = search_seg_locs[k]
                    kp_found = True
            
        if kp_found:        
            return kp_loc, search_seg
        else:
            search_seg -= 1


def show_pointclouds(clouds, WIN_NAME):
    nc = len(clouds)

    if nc == 2:
        rgb_plot = clouds[1].copy()
        cv2.imshow(WIN_NAME, rgb_plot)
        cv2.waitKey(100)
    else:
        rgb_plot_multi = []
        for i in range(nc):
            rgb_plot_multi.append(clouds[i][1].copy())
            cv2.imshow(WIN_NAME, rgb_plot_multi[i])
            cv2.waitKey(100)

#########################################
### find all keypoints
#########################################
def get_kp_locations(kp_names, exec_keypts, current_seg, cloud_topic):
    listener = ru.get_tf_listener()
    kp_locations = []
    #kp_xypixels = []
    
    seg_kps = []
    if kp_names[0] == "tip_transform":         
        while True:
            xyz_tfs, rgb_plots = get_kp_clouds(listener, cloud_topic, 30)
            needle_tip_loc = find_needle_tip(xyz_tfs, rgb_plots, WIN_NAME)
            if needle_tip_loc[2] > 1:   
                return needle_tip_loc              
            else: 
                print colorize("Didn't find a high enough point! Trying again","red", True, True)
        
    elif kp_names[0] in ["needle_end", "razor"]:
        while True:
            xyz_tfs, rgb_plots = get_kp_clouds(listener, cloud_topic, 30)        
            kp_loc, pts = find_red_block(xyz_tfs, rgb_plots, WIN_NAME)
            if pts > 0: 
                seg_kps.append(kp_loc)
                #kp_xypixels.append((0,0))
                break
            else: 
                print colorize("Couldn't find the keypoint! Trying again","red", True, True)             
    
    elif kp_names[0] == "needle_tip":
        while True:
            xyz_tfs, rgb_plots = get_kp_clouds(listener, cloud_topic, 30)
            kp_loc = find_needle_tip(xyz_tfs, rgb_plots, WIN_NAME)
            if kp_loc[2] > 0.8: 
                seg_kps.append(kp_loc)
                #kp_xypixels.append((0,0))
                break                
            else: 
                print colorize("Didn't find a high enough point! Trying again","red", True, True)              
            
    else:
        xyz_tf, rgb_plot = get_kp_clouds(listener, cloud_topic, 1)
        np.save("/tmp/xyz_tf.npy",xyz_tf)
        np.save("/tmp/rgb.npy",rgb_plot)
        for k in range(len(kp_names)):
            kp_loc = find_kp(kp_names[k], xyz_tf, rgb_plot, exec_keypts, current_seg, WIN_NAME)
            #kp_xypixels.append(kp_xypix)
            seg_kps.append(kp_loc)
                
    return seg_kps



#########################################
### find a single keypoint
#########################################
def find_kp(kp, xyz_tf, rgb_plot, past_keypts, current_seg, WIN_NAME):
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
    
    print colorize("Click on the center of the %s."%kp, 'red', bold=True)
    print colorize("If this keypont is occluded, select the image window and press any key", 'red', bold=True)
    
    gc = GetClick()
    cv2.setMouseCallback(WIN_NAME, gc.callback)
    while not gc.done:
        cv2.imshow(WIN_NAME, rgb_plot)
        k = cv2.waitKey(100)
        if k == -1:
            continue
        else:
            last_loc, found_seg = get_last_kp_loc(past_keypts, kp, current_seg)
            print kp, "found in segment %s at location %s"%(found_seg, last_loc)
            return last_loc

    row_kp = gc.x
    col_kp = gc.y
    
    cv2.circle(rgb_plot, (row_kp, col_kp), 5, (0, 0, 255), -1)
    cv2.imshow(WIN_NAME, rgb_plot)    
    cv2.waitKey(100)

    xyz_tf[np.isnan(xyz_tf)] = -2
    x, y, z = xyz_tf[col_kp, row_kp]

    print kp, "3d location", x, y, z

    #return (x, y, z), (col_kp, row_kp)
    return (x, y, z)


#########################################
### find needle
#########################################
def find_needle_tip(xyz_tfs, rgb_plots, WIN_NAME):

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
        cv2.setMouseCallback(WIN_NAME, gc.callback)
        while not gc.done:
            cv2.imshow(WIN_NAME, rgb_plot)
            cv2.waitKey(10)
        needle_rect_corners.append(gc.xy)

    xy_tl = np.array(needle_rect_corners).min(axis=0)
    xy_br = np.array(needle_rect_corners).max(axis=0)

    cv2.rectangle(rgb_plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0))
    cv2.imshow(WIN_NAME, rgb_plot)
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

    #cv2.circle(rgb_plots[ind], (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2)
    #cv2.imshow(WIN_NAME, rgb_plots[ind])
    #cv2.waitKey(100)
        

    cv2.circle(rgb_plot, (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2)
    cv2.imshow(WIN_NAME, rgb_plot)
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
def find_red_block(xyz_tfs, rgbs, WIN_NAME):

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
        cv2.setMouseCallback(WIN_NAME, gc.callback)
        while not gc.done:
            cv2.imshow(WIN_NAME, rgb_plot)
            cv2.waitKey(10)
        needle_rect_corners.append(gc.xy)

    xy_tl = np.array(needle_rect_corners).min(axis=0)
    xy_br = np.array(needle_rect_corners).max(axis=0)

    cv2.rectangle(rgb_plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0))
    cv2.imshow(WIN_NAME, rgb_plot)
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
    cv2.imshow(WIN_NAME, rgb_plot)
    cv2.waitKey(100)    

    print 'Needle end location', xyz_avg
 
    return xyz_avg, valid_pts