#import argparse
#parser = argparse.ArgumentParser()
#parser.add_argument("--mode", choices=["live","test"], default="live")
#parser.add_argument("--plot3d", action="store_true")
#args = parser.parse_args()

"""
# how to get a pcd file
# plug asus into local computer
import cloudprocpy # my homemade pcl python bindings. built with trajopt if BUILD_CLOUDPROC enabled
grabber=cloudprocpy.CloudGrabber()
xyzrgb = grabber.getXYZRGB()
xyzrgb.save("pr2_suture_scene.pcd")
"""

#import cv2
#import numpy as np
#import rospy
#import geometry_msgs.msg as gm
#import sensor_msgs.msg as sm
#import sys

#rospy.init_node("image_gui", disable_signals = True)
#rospy.init_node("image_gui")

#xyz = []
#xyz_tf = []
#rgb = []


#if args.mode == "test":
    #from jds_image_proc.pcd_io import load_xyzrgb
    #import openravepy

    #env = openravepy.Environment()
    #env.Load("robots/pr2-beta-static.zae")
    #robot = env.GetRobots()[0]
    #robot.SetDOFValues([1.5], [14])
    #hmat = robot.GetLink("narrow_stereo_gazebo_r_stereo_camera_optical_frame").GetTransform()

    #for i in range(5):

    #xyz_temp, rgb_temp = load_xyzrgb("pr2_suture_scene%s.pcd"%i)      
    #xyz.append(xyz_temp)
    #rgb.append(rgb_temp)

    #xyz1 = np.c_[xyz_temp.reshape(-1,3), np.ones((xyz_temp.size/3,1))]
    #xyz1_tf = np.dot(xyz1, hmat.T)
    #xyz_tf_temp = xyz1_tf[:,:3].reshape(xyz_temp.shape)
    #xyz_tf_temp[np.isnan(xyz_tf_temp)] = -2         

    #xyz_tf.append(xyz_tf_temp)   

    #del xyz_temp
    #del rgb_temp

    ##print 'xyz[0]', xyz[0]
    ##print 'xyz_tf[0]', xyz_tf[0]

#elif args.mode == "live":
    #import brett2.ros_utils as ru

    #listener = ru.get_tf_listener()

    #for i in range(5):
    #print "waiting for messages on cloud topic %s"%args.cloud_topic
    #msg = rospy.wait_for_message(args.cloud_topic, sm.PointCloud2)
    #print "got msg %s!"%i

    #xyz_temp, rgb_temp = ru.pc2xyzrgb(msg)
    #if (xyz_temp.shape[0] == 1 or xyz_temp.shape[1] == 1): raise Exception("needs to be an organized point cloud")

    #xyz.append(xyz_temp)
    #rgb.append(rgb_temp)

    #xyz_tf_temp = ru.transform_points(xyz_temp, listener, "base_footprint", "/camera_rgb_optical_frame")        
    #xyz_tf_temp[np.isnan(xyz_tf_temp)] = -2        
    #xyz_tf.append(xyz_tf_temp)
    ##print 'xyz_temp', xyz_temp    

    #del xyz_temp
    #del rgb_temp

    #rospy.sleep(1.0)       

    ##print 'xyz_tf[0]', xyz_tf[0]

#raw_input("Press enter to continue...")   


import cv2
import math
import numpy as np
from jds_image_proc.pcd_io import load_xyzrgb

cv2.namedWindow("rgb")

class GetClick:
    xy = None      
    done = False
    def callback(self, event, x, y, flags, param):
        if self.done:
            return
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.xy = (x,y)
            self.done = True

xyz, rgb = load_xyzrgb("suture_scene(findholescut)0.pcd")  

#orig = cv2.imread("c.jpg")
#img = orig.copy()
img = rgb.copy()

rect_corners = []

for i in xrange(2):
    gc = GetClick()
    cv2.setMouseCallback("rgb", gc.callback)
    while not gc.done:
        cv2.imshow("rgb", img)
        cv2.waitKey(10)
    rect_corners.append(gc.xy)

xy_tl = np.array(rect_corners).min(axis=0)
xy_br = np.array(rect_corners).max(axis=0)
cv2.rectangle(img, tuple(xy_tl), tuple(xy_br), (0, 255, 0)) 
cv2.imshow("rgb", img)
cv2.waitKey(100)

colmin, rowmin = xy_tl
colmax, rowmax = xy_br

h, w = img.shape[:2]
#print 'h,w', h, w
new_img = img[rowmin:rowmax , colmin:colmax]
new_img2 = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)
cv2.namedWindow("rgb2")
cv2.imshow("rgb2", new_img)

#raw_input("Press enter to continue...")

d_red = cv2.cv.RGB(150, 55, 65)
l_red = cv2.cv.RGB(250, 200, 200)

detector = cv2.FeatureDetector_create('MSER')
fs = detector.detect(new_img2)
fs.sort(key = lambda x: -x.size)

def supress(x):
    for f in fs:
        distx = f.pt[0] - x.pt[0]
        disty = f.pt[1] - x.pt[1]
        dist = math.sqrt(distx*distx + disty*disty)
        print 'f.size', f.size
        print 'x.size', x.size
        print 'dist', dist        
        if (f.size > x.size) and (dist < f.size/2):  
            print 'f.size', f.size
            print 'x.size', x.size
            print 'dist', dist            
            return True


sfs = [x for x in fs if not supress(x)]


for f in sfs:
    cv2.circle(new_img, (int(f.pt[0]), int(f.pt[1])), int(f.size/2), d_red, 2, cv2.CV_AA)
    cv2.circle(new_img, (int(f.pt[0]), int(f.pt[1])), int(f.size/2), l_red, 1, cv2.CV_AA)

h, w = new_img.shape[:2]
#vis = np.zeros((h, w*2+5), np.uint8)
vis = np.zeros((h, w), np.uint8)
vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
vis[:h, :w] = new_img
#vis[:h, w+5:w*2+5] = new_img

cv2.imshow("image", vis)
#cv2.imwrite("c_o.jpg", vis)
cv2.waitKey()
cv2.destroyAllWindows()








#row_needle = []
#col_needle = []

## extract depths from drawn rectangle
#for i in range(5):
    #z_rectangle = xyz_tf[i][rowmin:rowmax, colmin:colmax, 2] # z values in extracted rectangle
    #row_needle_temp, col_needle_temp = np.unravel_index(z_rectangle.argmax(), z_rectangle.shape)

    #col_needle_temp += colmin # since these indices are from small rectangle
    #row_needle_temp += rowmin

    #row_needle.append(row_needle_temp)
    #col_needle.append(col_needle_temp)

    #del z_rectangle
    #del col_needle_temp
    #del row_needle_temp


#xneedle = np.zeros((5))
#yneedle = np.zeros((5))
#zneedle = np.zeros((5))

#for i in range(5):
    #xneedle[i], yneedle[i], zneedle[i] = xyz_tf[i][row_needle[i], col_needle[i]]

#print "xyz needle point %s"%i, xneedle, yneedle, zneedle

#ind = np.argmax(zneedle)

#max_needle = []
#max_needle.append(xneedle[ind])
#max_needle.append(yneedle[ind])
#max_needle.append(zneedle[ind])

##rgb_plot_best = rgb[ind].copy()
##cv2.namedWindow("best_rgb")
##cv2.imshow("best_rgb", rgb_plot_best)

#cv2.circle(rgb_plot, (col_needle[ind], row_needle[ind]), 3, (255, 0, 0), 2) ## this doesn't seem to be working...

##save needle location to np file
#np.save('/home/mallory/mallory_workspace/suturing/python/iros/needleLoc.npy', np.array(max_needle))
#print 'needle location saved!'


# plot the point cloud with a circle around "highest" point    
#if args.plot3d:
    #from mayavi import mlab
    #x,y,z = xyz_tf[ind][~np.isnan(xyz_tf[ind][:,:,0])].T    
    #mlab.points3d(x,y,z,color=(1,0,0), mode="2dvertex")
    #mlab.points3d([max_needle[0]], [max_needle[1]], [max_needle[2]], color=(0,1,0), mode="sphere", scale_factor=.04, opacity=.2)
    #mlab.show()
#else:
    #rospy.sleep(1)  
    #sys.exit()
    #while True:
    #    cv2.imshow("rgb",rgb_plot)
    #    cv2.imshow("depth", depth_plot)
    #    cv2.waitKey(10)

#raw_input("press enter when done")        
