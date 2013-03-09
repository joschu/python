import cv
import cv2
import math
import numpy as np
from jds_image_proc.pcd_io import load_xyzrgb
from jds_utils.colorize import colorize

#xyz, rgb = load_xyzrgb("suture_scene(holes).pcd")  
#xyz, rgb = load_xyzrgb("suture_scene(findholescut)0.pcd")
xyz, rgb = load_xyzrgb("suture_scene(findneedle)0.pcd")
img = rgb.copy()
img2 = rgb.copy()
gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

d_red = cv2.cv.RGB(150, 55, 65)
l_red = cv2.cv.RGB(250, 200, 200)
d_blue = cv2.cv.RGB(40, 55, 200)
clr = cv2.cv.RGB(0, 0, 0)

cv2.namedWindow("rgb")
cv2.imshow("rgb", img)
cv2.waitKey(10)


#cv2.HoughLinesP(image, rho, theta, threshold[, lines[, minLineLength[, maxLineGap]]]) lines
#cv2.HoughLinesP( img, lines, 1, CV_PI/180, 80, 30, 10 )

edges = cv2.Canny(gray_img, 80, 240)
cv2.imshow("edges", edges)
cv2.waitKey(100)

lines = cv2.HoughLinesP(edges, 1, cv.CV_PI/360, 1, minLineLength = 50, maxLineGap = 5)

vert_lines = [l for l in lines[0] if (math.atan2(abs(l[0] - l[2]) , abs(l[1] - l[3])) == 0.0)]
       
for l in vert_lines:
    cv2.line(img, (l[0], l[1]), (l[2], l[3]), clr, 2)        

#cv2.line(img, (vert_lines[0][0], vert_lines[0][1]), (vert_lines[0][2], vert_lines[0][3]), clr, 2)

cv2.imshow("rgb with vertical lines", img)
cv2.waitKey(100)

for l in lines[0]:
    cv2.line(img2, (l[0], l[1]), (l[2], l[3]), clr, 2)        

cv2.imshow("rgb with all lines", img2)
cv2.waitKey(100)
   
raw_input("Press enter to finish...")    
cv2.destroyAllWindows()

     
