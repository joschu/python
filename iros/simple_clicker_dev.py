import argparse
parser = argparse.ArgumentParser()
parser.add_argument("mode", choices=["live","test"], default="live")
parser.add_argument("object", choices=["holes_cut", "needle"])
parser.add_argument("cloud_topic", default="/drop/points")
parser.add_argument("plot3d", action="store_true")
args = parser.parse_args()


# how to get a pcd file
#import cloudprocpy 
#grabber=cloudprocpy.CloudGrabber()
#xyzrgb = grabber.getXYZRGB()
#xyzrgb.save("pr2_suture_scene(needle_finding).pcd")

#raw_input("got point cloud. press enter to continue")

import cv2
import numpy as np
import rospy
import openravepy
import sys

import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
from jds_image_proc.pcd_io import load_xyzrgb
from jds_utils.colorize import colorize
import brett2.ros_utils as ru

rospy.init_node("image_gui")

#########################################
### find holes and cut
#########################################
def find_holes_cut(plot):
  
    hole1_center = []
    hole2_center = []
    cut_endpoints = []

    print colorize("click on the centers of the two relevant holes", 'red', bold=True)
    
    for i in xrange(2):
        gc = GetClick()
        cv2.setMouseCallback("rgb", gc.callback)
        while not gc.done:
            cv2.imshow("rgb", plot)
            cv2.waitKey(10)
        if i == 1:
            hole1_center.append(gc.x)
            hole1_center.append(gc.y)
        else:
            hole2_center.append(gc.x)
            hole2_center.append(gc.y)

    cv2.circle(plot, (hole1_center[0], hole1_center[1]), 10, (255, 0, 0), 2)
    cv2.circle(plot, (hole2_center[0], hole2_center[1]), 10, (255, 0, 0), 2)
    cv2.imshow("rgb", plot)
    cv2.waitKey(100)

    x_hole1, y_hole1, z_hole1 = xyz_tf[hole1_center[0], hole1_center[1]]
    x_hole2, y_hole2, z_hole2 = xyz_tf[hole2_center[0], hole2_center[1]]
    
    print 'hole1 3d location', x_hole1, y_hole1, z_hole1
    print 'hole2 3d location', x_hole2, y_hole2, z_hole2
    
    raw_input("press enter to continue")
        
    ### find cut
    
    print colorize("click at the top and bottom of the cut line", 'red', bold=True)

    for i in xrange(2):
        gc = GetClick()
        cv2.setMouseCallback("rgb", gc.callback)
        while not gc.done:
            cv2.imshow("rgb", plot)
            cv2.waitKey(10)
        cut_endpoints.append(gc.xy)

    xy_t = np.array(cut_endpoints).min(axis=0)
    xy_b = np.array(cut_endpoints).max(axis=0)

    cv2.line(plot, tuple(xy_t), tuple(xy_b), (0, 255, 0), 2)
    cv2.imshow("rgb", plot)
    cv2.waitKey(100)
    
    x1_cut, y1_cut, z1_cut = xyz_tf[tuple(xy_t)]
    x2_cut, y2_cut, z2_cut = xyz_tf[tuple(xy_b)]
    

    #midpoint of line
    mid_y = (xy_t[1] - xy_b[1])/2    
    mid_plot_x = xy_b[0]
    mid_plot_y = xy_t[1] - mid_y
    
    cv2.circle(plot, (mid_plot_x, mid_plot_y), 5, (0, 0, 255), -1)
    cv2.imshow("rgb", plot)
    cv2.waitKey(100)
    
    mid_ptx, mid_pty, mid_ptz = xyz_tf[mid_plot_x, mid_plot_y]
    
    print 'top of cut 3d', x1_cut, y1_cut, z1_cut
    print 'bottom of cut 3d', x2_cut, y2_cut, z2_cut
    print 'middle of cut 3d', mid_ptx, mid_pty, mid_ptz
    
    raw_input("press enter to continue")


#########################################
### find needle
#########################################
def find_needle(plot):
  
    needle_rect_corners = []

    print colorize("click at the corners of a rectangle which encompasses the needle tip", 'red', bold=True)

    for i in xrange(2):
        gc = GetClick()
        cv2.setMouseCallback("rgb", gc.callback)
        while not gc.done:
            cv2.imshow("rgb", plot)
            cv2.waitKey(10)
        needle_rect_corners.append(gc.xy)
    
    xy_tl = np.array(needle_rect_corners).min(axis=0)
    xy_br = np.array(needle_rect_corners).max(axis=0)

    cv2.rectangle(plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0))
    cv2.imshow("rgb", plot)
    cv2.waitKey(100)

    colmin, rowmin = xy_tl
    colmax, rowmax = xy_br

    row_needle = []
    col_needle = []

    # extract depths from drawn rectangle
    for i in range(3):
        z_rectangle = xyz_tf[i][rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle
        row_needle_temp, col_needle_temp = np.unravel_index(z_rectangle.argmax(), z_rectangle.shape)

        col_needle_temp += colmin # since these indices are from small rectangle
        row_needle_temp += rowmin

        row_needle.append(row_needle_temp)
        col_needle.append(col_needle_temp)
    
        del z_rectangle
        del col_needle_temp
        del row_needle_temp


    xneedle = np.zeros((3))
    yneedle = np.zeros((3))
    zneedle = np.zeros((3))

    for i in range(3):
        xneedle[i], yneedle[i], zneedle[i] = xyz_tf[i][row_needle[i], col_needle[i]]
    
    print "xyz needle point %s"%i, xneedle, yneedle, zneedle

    ind = np.argmax(zneedle)

    max_needle = []
    max_needle.append(xneedle[ind])
    max_needle.append(yneedle[ind])
    max_needle.append(zneedle[ind])
    
    cv2.circle(rgb_plot, (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2)
    cv2.imshow("rgb", rgb_plot)
    cv2.waitKey(100)   

    # plot the point cloud with a circle around "highest" point    
    from mayavi import mlab
    x,y,z = xyz_tf[ind][~np.isnan(xyz_tf[ind][:,:,0])].T    
    mlab.points3d(x,y,z,color=(1,0,0), mode="2dvertex")
    mlab.points3d([max_needle[0]], [max_needle[1]], [max_needle[2]], color=(0,1,0), mode="sphere", scale_factor=.04, opacity=.2)
    mlab.show()
    
    raw_input("press enter when done")        

#########################################
### clicking set-up
#########################################

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

cv2.namedWindow("rgb")
    
xyz = []
xyz_tf = []
rgb = []

if args.object == "needle":
    if args.mode == "test":
        for i in range(3):
            xyz_temp, rgb_temp = load_xyzrgb("pr2_suture_scene(needle_finding)%s.pcd"%i)      
            xyz.append(xyz_temp)
            rgb.append(rgb_temp)

            env = openravepy.Environment()
            env.Load("robots/pr2-beta-static.zae")
            robot = env.GetRobots()[0]
            robot.SetDOFValues([1.5], [14])
            hmat = robot.GetLink("narrow_stereo_gazebo_r_stereo_camera_optical_frame").GetTransform()            
        
            xyz1 = np.c_[xyz_temp.reshape(-1,3), np.ones((xyz_temp.size/3,1))]
            xyz1_tf = np.dot(xyz1, hmat.T)
            xyz_tf_temp = xyz1_tf[:,:3].reshape(xyz_temp.shape)
            xyz_tf_temp[np.isnan(xyz_tf_temp)] = -2         
        
            xyz_tf.append(xyz_tf_temp)   
        
            del xyz_temp
            del rgb_temp  
   
      
    elif args.mode == "live":
        listener = ru.get_tf_listener()
        for i in range(5):
            print "waiting for messages on cloud topic %s"%args.cloud_topic
            msg = rospy.wait_for_message(args.cloud_topic, sm.PointCloud2)
            print "got msg %s!"%i
        
            xyz_temp, rgb_temp = ru.pc2xyzrgb(msg)
            if (xyz_temp.shape[0] == 1 or xyz_temp.shape[1] == 1): raise Exception("needs to be an organized point cloud")

            xyz.append(xyz_temp)
            rgb.append(rgb_temp)
        
            xyz_tf_temp = ru.transform_points(xyz_temp, listener, "base_footprint", "/camera_rgb_optical_frame")        
            xyz_tf_temp[np.isnan(xyz_tf_temp)] = -2        
            xyz_tf.append(xyz_tf_temp)    
        
            del xyz_temp
            del rgb_temp
        
            rospy.sleep(1.0) 

    rgb_plot = rgb[0].copy()
    find_needle(rgb_plot)

elif args.object == "holes_cut":
    if args.mode == "test":

        env = openravepy.Environment()
        env.Load("robots/pr2-beta-static.zae")
        robot = env.GetRobots()[0]
        robot.SetDOFValues([1.5], [14])
        hmat = robot.GetLink("narrow_stereo_gazebo_r_stereo_camera_optical_frame").GetTransform()
    
        xyz, rgb = load_xyzrgb("suture_scene(findholescut).pcd")      
        
        xyz1 = np.c_[xyz.reshape(-1,3), np.ones((xyz.size/3,1))]
        xyz1_tf = np.dot(xyz1, hmat.T)
        xyz_tf = xyz1_tf[:,:3].reshape(xyz.shape)
        xyz_tf[np.isnan(xyz_tf)] = -2         
   
    elif args.mode == "live":
        listener = ru.get_tf_listener()
        print "waiting for messages on cloud topic %s"%args.cloud_topic
        msg = rospy.wait_for_message(args.cloud_topic, sm.PointCloud2)
        
        xyz, rgb = ru.pc2xyzrgb(msg)
        if (xyz.shape[0] == 1 or xyz.shape[1] == 1): raise Exception("needs to be an organized point cloud")

        xyz.append(xyz)
        rgb.append(rgb)
        
        xyz_tf = ru.transform_points(xyz, listener, "base_footprint", "/camera_rgb_optical_frame")        
        xyz_tf[np.isnan(xyz_tf)] = -2

    rgb_plot = rgb.copy()

    find_holes_cut(rgb_plot)



