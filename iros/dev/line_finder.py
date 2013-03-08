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

#cv2.HoughLinesP(image, rho, theta, threshold[, lines[, minLineLength[, maxLineGap]]]) lines
#cv2.HoughLinesP( img, lines, 1, CV_PI/180, 80, 30, 10 )
edges = cv2.Canny(gray_img, 80, 120)
lines = cv2.HoughLinesP(edges, 1, math.pi/2, 2, None, 30, 1);

       
for l in lines[0]:
    cv2.line(img, (l[0], l[1]), (l[2], l[3]), d_red, 2)
    cv2.imshow("rgb with lines", img)
    cv2.waitKey(10)         
    #raw_input("Circle %i. Press enter to continue..."%d) 

cv2.imshow("rgb with lines", img)
cv2.waitKey(100)
raw_input("Press enter to finish...")    
cv2.destroyAllWindows()

     
