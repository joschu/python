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

#xyz, rgb = load_xyzrgb("suture_scene(holes).pcd")  
#xyz, rgb = load_xyzrgb("suture_scene(findholescut)0.pcd")
xyz, rgb = load_xyzrgb("suture_scene(findneedle)0.pcd")
img = rgb.copy()
img2 = rgb.copy()

d_red = cv2.cv.RGB(250, 55, 65)
l_red = cv2.cv.RGB(250, 200, 200)
d_blue = cv2.cv.RGB(40, 55, 200)

cv2.namedWindow("rgb")
cv2.imshow("rgb", img)
cv2.waitKey(10)

SIZEMIN = 20
SIZEMAX = 33

COLMIN = 200
COLMAX = 500

gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#########################################
### MSER Detector Implementation
#########################################

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

horiz_pairs = []
diag_pairs = []
for i in range(len(circles)/2):
    horiz_pairs.append([])
    horiz_pairs[i].append(circles[2*i])
    horiz_pairs[i].append(circles[(2*i)+1])

for i in range(len(horiz_pairs)):
    horiz_pairs[i].sort(key = lambda c: c.pt[0])

for i in range(len(horiz_pairs)-1):
    diag_pairs.append([])
    diag_pairs[i].append(horiz_pairs[i][1])
    diag_pairs[i].append(horiz_pairs[i+1][0])
      
#print 'num circles', len(circles)
#print 'num horiz_pairs', len(horiz_pairs)
#print 'num diag_pairs', len(diag_pairs)

cv2.namedWindow("rgb with MSER circles")
cv2.imshow("rgb with MSER circles", img)
cv2.waitKey(100)

n = 0
for p in range(len(horiz_pairs)):
    for c in horiz_pairs[p]:
        n += 1
        cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_red, 2, cv2.CV_AA)
        cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_blue, 1, cv2.CV_AA)   
        cv2.imshow("rgb with MSER circles", img)
        cv2.waitKey(10)         
        #raw_input("Circle %i. Press enter to continue..."%n)  
        
#d = 0
#for p in range(len(diag_pairs)):
    #for c in diag_pairs[p]:
        #d += 1
        #cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_red, 2, cv2.CV_AA)
        #cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_blue, 1, cv2.CV_AA)   
        #cv2.imshow("rgb with circles", img)
        #cv2.waitKey(10)         
        #raw_input("Circle %i. Press enter to continue..."%d) 


#########################################
### Hough Transform Implementation
#########################################
#cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) 
#circles = cv2.HoughCircles(img,cv2.cv.CV_HOUGH_GRADIENT,1,10,param1=100,param2=30,minRadius=5,maxRadius=20)
#for i in circles[0,:]:
        #cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),1)
        
circles = cv2.HoughCircles(gray_img, cv.CV_HOUGH_GRADIENT, 1, 50, param1=240, param2=20, minRadius=10, maxRadius=30)
circles = np.uint16(np.around(circles))

for c in circles[0,:]:
    cv2.circle(img2, (c[0], c[1]), c[2], d_red, 2, cv2.CV_AA)
    cv2.circle(img2, (c[0], c[1]), c[2], d_blue, 1, cv2.CV_AA)   

cv2.namedWindow("rgb with hough circles")    
cv2.imshow("rgb with hough circles", img2)
cv2.waitKey(100)   


cv2.imshow("rgb with hough circles", img2)
cv2.waitKey(100)
raw_input("Press enter to finish...")    
cv2.destroyAllWindows()

     
