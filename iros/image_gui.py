import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--mode", choices=["live","test"], default="live")
parser.add_argument("--cloud_topic", default="/drop/points")
parser.add_argument("--plot3d", action="store_true")
args = parser.parse_args()

"""
# how to get a pcd file
import cloudprocpy # my homemade pcl python bindings. built with trajopt if BUILD_CLOUDPROC enabled
grabber=cloudprocpy.CloudGrabber()
xyzrgb = grabber.getXYZRGB()
xyzrgb.save("pr2_suture_scene.pcd")
"""

import cv2
import numpy as np


if args.mode == "test":
    from jds_image_proc.pcd_io import load_xyzrgb
    import openravepy

    xyz, rgb = load_xyzrgb("pr2_suture_scene.pcd")
    env = openravepy.Environment()
    env.Load("robots/pr2-beta-static.zae")
    robot = env.GetRobots()[0]
    robot.SetDOFValues([1.5], [14])
    hmat = robot.GetLink("narrow_stereo_gazebo_r_stereo_camera_optical_frame").GetTransform()
    xyz1 = np.c_[xyz.reshape(-1,3), np.ones((xyz.size/3,1))]
    xyz1_tf = np.dot(xyz1, hmat.T)
    xyz_tf = xyz1_tf[:,:3].reshape(xyz.shape) 

elif args.mode == "live":
    import brett2.ros_utils as ru
    import rospy
    import sensor_msgs.msg as sm

    rospy.init_node("image_gui", disable_signals = True)
    listener = ru.get_tf_listener()
    print "waiting for message on cloud topic %s"%args.cloud_topic
    msg = rospy.wait_for_message(args.cloud_topic, sm.PointCloud2)
    print "got it!"
    xyz, rgb = ru.pc2xyzrgb(msg)
    if (xyz.shape[0] == 1 or xyz.shape[1] == 1): raise Exception("needs to be an organized point cloud")
    xyz_tf = ru.transform_points(xyz, listener, "base_footprint", "/camera_rgb_optical_frame")

xyz_tf[np.isnan(xyz_tf)] = -2
rgb_plot = rgb.copy()
depth_plot = xyz_tf[:,:,2].astype('float')
depth_plot[np.isnan(depth_plot)] = 0
depth_plot -= depth_plot.min()
depth_plot = (depth_plot * (255 / depth_plot.max())).astype('uint8')

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

# extract depths from drawn rectangle  ##ADD STUFF HERE to make this work better
colmin, rowmin = xy_tl
colmax, rowmax = xy_br

z_rectangle = xyz_tf[rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle

row_needle, col_needle = np.unravel_index(z_rectangle.argmax(), z_rectangle.shape)
col_needle += colmin # since these indices are from small rectangle
row_needle += rowmin
cv2.circle(rgb_plot, (col_needle, row_needle), 3, (255, 0, 0), 2) ## this doesn't seem to be working...

# plot the point cloud with a circle around "highest" point    
if args.plot3d:
    from mayavi import mlab
    x,y,z = xyz_tf[~np.isnan(xyz_tf[:,:,0])].T    
    mlab.points3d(x,y,z,color=(1,0,0), mode="2dvertex")
    xneedle,yneedle,zneedle = xyz_tf[row_needle, col_needle]
    print "xyz needle point", xneedle, yneedle, zneedle
    mlab.points3d([xneedle], [yneedle], [zneedle], color=(0,1,0), mode="sphere", scale_factor=.04, opacity=.2)
    mlab.show()
else:    
    while True:
        cv2.imshow("rgb",rgb_plot)
        #cv2.imshow("depth", depth_plot)
        cv2.waitKey(10)
