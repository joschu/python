#import cv2
#import numpy as np
#import rospy
#import geometry_msgs.msg as gm
#import sensor_msgs.msg as sm
#import sys


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

import cv
import cv2
import math
import numpy as np
from jds_image_proc.pcd_io import load_xyzrgb
from jds_utils.colorize import colorize

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

img = rgb.copy()

d_red = cv2.cv.RGB(150, 55, 65)
l_red = cv2.cv.RGB(250, 200, 200)
d_blue = cv2.cv.RGB(65, 55, 150)

#minRadius = 21
#maxRadius = 33
#storage = cv.CreateMat(w, 1, cv.CV_32FC3)
#cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) 
#cv.HoughCircles(processed, storage, cv.CV_HOUGH_GRADIENT, 2, 32.0, HIGH, LOW)
#for i in range(0,len(np.asarray(storage))):
#    cv.Circle(gray_img, ( int(np.asarray(storage)[i][0][0]), int(np.asarray(storage)[i][0][1]) ), int(np.asarray(storage)[i][0][2]), cv.CV_RGB(255, 0, 0), 2, 8, 0 )
#cv2.HoughCircles(gray_img, int(np.asarray(storage)), cv.CV_HOUGH_GRADIENT, 1, 42, minRadius, maxRadius)

#for i in range(0, len(np.asarray(storage))):
#    print "circle #%d" %i
#    radius = int(np.asarray(storage)[i][0][2])
#    x = int(np.asarray(storage)[i][0][0])
#    y = int(np.asarray(storage)[i][0][1])
#    center = (x, y)
#    print 'radius', radius

#raw_input("Press enter to continue...")


cv2.namedWindow("rgb")
rect_corners = []

print colorize("click at the corners of the relevant area", 'red')

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

new_img = img[rowmin:rowmax , colmin:colmax]
gray_img = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)

detector = cv2.FeatureDetector_create('MSER')
fs = detector.detect(gray_img)
fs.sort(key = lambda x: -x.size)

def supress(x):
    for f in fs:
        distx = f.pt[0] - x.pt[0]
        disty = f.pt[1] - x.pt[1]
        dist = math.sqrt(distx*distx + disty*disty)
        if (f.size > x.size) and (dist < f.size/2):
            return True


sfs = [x for x in fs if not supress(x)]
print 'num circles', len(sfs)


for f in sfs:
    #cv2.circle(new_img, (int(f.pt[0]), int(f.pt[1])), int(f.size/2), d_red, 2, cv2.CV_AA)
    #cv2.circle(new_img, (int(f.pt[0]), int(f.pt[1])), int(f.size/2), l_red, 1, cv2.CV_AA)
    if (f.size > 21) and (f.size < 33):
        cv2.circle(new_img, (int(f.pt[0]), int(f.pt[1])), int(f.size/2), d_red, 2, cv2.CV_AA)
        cv2.circle(new_img, (int(f.pt[0]), int(f.pt[1])), int(f.size/2), d_blue, 1, cv2.CV_AA)

cv2.namedWindow("rgb with circles")
cv2.imshow("rgb with circles", new_img)
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
