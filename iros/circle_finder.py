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

    #print 'xyz[0]', xyz[0]
    #print 'xyz_tf[0]', xyz_tf[0]

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

xyz, rgb = load_xyzrgb("suture_scene(holes).pcd")  
img = rgb.copy()

d_red = cv2.cv.RGB(150, 55, 65)
l_red = cv2.cv.RGB(250, 200, 200)
d_blue = cv2.cv.RGB(40, 55, 200)

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
cv2.imshow("rgb", img)
cv2.waitKey(10)

#rect_corners = []

#print colorize("click at the corners of the relevant area", 'red')

#for i in xrange(2):
    #gc = GetClick()
    #cv2.setMouseCallback("rgb", gc.callback)
    #while not gc.done:
        #cv2.imshow("rgb", img)
        #cv2.waitKey(10)
    #rect_corners.append(gc.xy)

#xy_tl = np.array(rect_corners).min(axis=0)
#xy_br = np.array(rect_corners).max(axis=0)
#cv2.rectangle(img, tuple(xy_tl), tuple(xy_br), (0, 255, 0)) 
#cv2.imshow("rgb", img)
#cv2.waitKey(100)

#colmin, rowmin = xy_tl
#colmax, rowmax = xy_br
#print 'colmin,colmax', colmin, colmax

SIZEMIN = 21
SIZEMAX = 33

COLMIN = 200
COLMAX = 500

gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

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

def fil(f):
    if (f.size > SIZEMIN) and (f.size < SIZEMAX) and (f.pt[0] > COLMIN) and (f.pt[0] < COLMAX):
        return True


sfs = [x for x in fs if not supress(x)]
circles = [c for c in sfs if fil(c)]
circles.sort(key = lambda c: c.pt[1], reverse=True)

pairs = []
for i in range(len(circles)/2):
    pairs.append([])
    pairs[1]([c for c in circles[(2*i):(2*i)+1]])

for i in range(len(pairs)):
    pairs[i].sort(key = lambda c: c.pt[0])

print 'num circles', len(circles)
print 'num pairs', len(pairs)

cv2.namedWindow("rgb with circles")
cv2.imshow("rgb with circles", img)
cv2.waitKey(100)


n = 0
for p in range(len(pairs)):
    print 'outer'
    for c in pairs[p]:
        print 'inner'
        n += 1
        cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_red, 2, cv2.CV_AA)
        cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_blue, 1, cv2.CV_AA)   
        cv2.imshow("rgb with circles", img)
        cv2.waitKey(10)         
        raw_input("Circle %i. Press enter to continue..."%n)    

cv2.imshow("rgb with circles", img)
cv2.waitKey()
cv2.destroyAllWindows()

     
