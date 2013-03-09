import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["live","test"], default="live")
parser.add_argument("--cloud_topic", default="/drop/points")
parser.add_argument("--plot3d", action="store_true")
args = parser.parse_args()

"""
# how to get a pcd file
# plug asus into local computer
import cloudprocpy # my homemade pcl python bindings. built with trajopt if BUILD_CLOUDPROC enabled
grabber=cloudprocpy.CloudGrabber()
xyzrgb = grabber.getXYZRGB()
xyzrgb.save("pr2_suture_scene.pcd")
"""

import cv2
import numpy as np
import rospy
import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
import sys

#rospy.init_node("image_gui", disable_signals = True)
rospy.init_node("image_gui")

xyz = []
xyz_tf = []
rgb = []


if args.mode == "test":
    from jds_image_proc.pcd_io import load_xyzrgb
    import openravepy

    env = openravepy.Environment()
    env.Load("robots/pr2-beta-static.zae")
    robot = env.GetRobots()[0]
    robot.SetDOFValues([1.5], [14])
    hmat = robot.GetLink("narrow_stereo_gazebo_r_stereo_camera_optical_frame").GetTransform()
    
    for i in range(5):
        
        xyz_temp, rgb_temp = load_xyzrgb("pr2_suture_scene%s.pcd"%i)      
        xyz.append(xyz_temp)
        rgb.append(rgb_temp)
        
        xyz1 = np.c_[xyz_temp.reshape(-1,3), np.ones((xyz_temp.size/3,1))]
        xyz1_tf = np.dot(xyz1, hmat.T)
        xyz_tf_temp = xyz1_tf[:,:3].reshape(xyz_temp.shape)
        xyz_tf_temp[np.isnan(xyz_tf_temp)] = -2         
        
        xyz_tf.append(xyz_tf_temp)   
        
        del xyz_temp
        del rgb_temp
    
    #print 'xyz[0]', xyz[0]
    #print 'xyz_tf[0]', xyz_tf[0]

elif args.mode == "live":
    import brett2.ros_utils as ru

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
        #print 'xyz_temp', xyz_temp    
        
        del xyz_temp
        del rgb_temp
        
        rospy.sleep(1.0)       

    #print 'xyz_tf[0]', xyz_tf[0]

#raw_input("Press enter to continue...")   

rgb_plot = rgb[0].copy()

#depth_plot = xyz_tf[:,:,2].astype('float')
#depth_plot[np.isnan(depth_plot)] = 0
#depth_plot -= depth_plot.min()
#depth_plot = (depth_plot * (255 / depth_plot.max())).astype('uint8')

class GetClick:
    xy = None      
    done = False
    def callback(self, event, x, y, flags, param):
        if self.done:
            return
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.xy = (x,y)
            self.done = True

cv2.namedWindow("rgb")
rect_corners = []
for i in xrange(2):
    gc = GetClick()
    cv2.setMouseCallback("rgb", gc.callback)
    while not gc.done:
        cv2.imshow("rgb", rgb_plot)
        cv2.waitKey(10)
    rect_corners.append(gc.xy)
    
xy_tl = np.array(rect_corners).min(axis=0)
xy_br = np.array(rect_corners).max(axis=0)
cv2.rectangle(rgb_plot, tuple(xy_tl), tuple(xy_br), (0, 255, 0)) ## this doesn't seem to be working...

colmin, rowmin = xy_tl
colmax, rowmax = xy_br

row_needle = []
col_needle = []

# extract depths from drawn rectangle
for i in range(5):
    z_rectangle = xyz_tf[i][rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle
    row_needle_temp, col_needle_temp = np.unravel_index(z_rectangle.argmax(), z_rectangle.shape)

    col_needle_temp += colmin # since these indices are from small rectangle
    row_needle_temp += rowmin

    row_needle.append(row_needle_temp)
    col_needle.append(col_needle_temp)
    
    del z_rectangle
    del col_needle_temp
    del row_needle_temp


xneedle = np.zeros((5))
yneedle = np.zeros((5))
zneedle = np.zeros((5))

for i in range(5):
    xneedle[i], yneedle[i], zneedle[i] = xyz_tf[i][row_needle[i], col_needle[i]]
    
print "xyz needle point %s"%i, xneedle, yneedle, zneedle

ind = np.argmax(zneedle)

max_needle = []
max_needle.append(xneedle[ind])
max_needle.append(yneedle[ind])
max_needle.append(zneedle[ind])

#rgb_plot_best = rgb[ind].copy()
#cv2.namedWindow("best_rgb")
#cv2.imshow("best_rgb", rgb_plot_best)

cv2.circle(rgb_plot, (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2) ## this doesn't seem to be working...

#save needle location to np file
np.save('/home/mallory/mallory_workspace/suturing/python/iros/needleLoc.npy', np.array(max_needle))
print 'needle location saved!'
    

# plot the point cloud with a circle around "highest" point    
if args.plot3d:
    from mayavi import mlab
    x,y,z = xyz_tf[ind][~np.isnan(xyz_tf[ind][:,:,0])].T    
    mlab.points3d(x,y,z,color=(1,0,0), mode="2dvertex")
    mlab.points3d([max_needle[0]], [max_needle[1]], [max_needle[2]], color=(0,1,0), mode="sphere", scale_factor=.04, opacity=.2)
    mlab.show()
#else:
    #rospy.sleep(1)  
    #sys.exit()
    #while True:
    #    cv2.imshow("rgb",rgb_plot)
    #    cv2.imshow("depth", depth_plot)
    #    cv2.waitKey(10)
    
raw_input("press enter when done")        
#rospy.spin()
