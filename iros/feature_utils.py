import cv
import cv2
import math
import numpy as np
import os
import os.path as osp
#from jds_image_proc.pcd_io import load_xyzrgb

def automatic_find_holes(img, task):
    #xyz, rgb = load_xyzrgb("suture_scene(holes).pcd")  
    #img = rgb.copy()
    
    SIZEMIN = 21
    SIZEMAX = 33
    
    COLMIN = 200
    COLMAX = 500    
    
    d_red = cv2.cv.RGB(150, 55, 65)
    l_red = cv2.cv.RGB(250, 200, 200)
    d_blue = cv2.cv.RGB(40, 55, 200)
    
    cv2.namedWindow("rgb")
    cv2.imshow("rgb", img)
    cv2.waitKey(10)
    
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
          
    print 'num circles', len(circles)
    print 'num horiz_pairs', len(horiz_pairs)
    print 'num diag_pairs', len(diag_pairs)
    
    cv2.namedWindow("rgb with circles")
    cv2.imshow("rgb with circles", img)
    cv2.waitKey(100)
    
    for p in range(len(horiz_pairs)):
        for c in horiz_pairs[p]:
            cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_red, 2, cv2.CV_AA)
            cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_blue, 1, cv2.CV_AA)   
            cv2.imshow("rgb with circles", img)
            cv2.waitKey(10)         
            #raw_input("Press enter for the next circle...")  
            
    #d = 0
    #for p in range(len(diag_pairs)):
        #for c in diag_pairs[p]:
            #d += 1
            #cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_red, 2, cv2.CV_AA)
            #cv2.circle(img, (int(c.pt[0]), int(c.pt[1])), int(c.size/2), d_blue, 1, cv2.CV_AA)   
            #cv2.imshow("rgb with circles", img)
            #cv2.waitKey(10)         
            #raw_input("Circle %i. Press enter to continue..."%d) 
    
    cv2.imshow("rgb with circles", img)
    cv2.waitKey(100)
    raw_input("Press enter to finish...")    
    cv2.destroyAllWindows()
    
    if task == 'InterruptedSuture': 
        return horiz_pairs
    elif task == 'RunningSuture':    
        return diag_pairs
    else:
        return None



def automatic_find_cut(img):
    #xyz, rgb = load_xyzrgb("suture_scene(holes).pcd")  
    #img = rgb.copy()
    
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 80, 240)
    cv2.imshow("edges", edges)
    cv2.waitKey(100)
    
    lines = cv2.HoughLinesP(edges, 1, cv.CV_PI/360, 1, minLineLength = 50, maxLineGap = 5)
    
    vert_lines = [l for l in lines[0] if (math.atan2(abs(l[0] - l[2]) , abs(l[1] - l[3])) == 0.0)]
           
    for l in vert_lines:
        cv2.line(img, (l[0], l[1]), (l[2], l[3]), clr, 2)        
    
    #cv2.imshow("rgb with vertical lines", img)
    #cv2.waitKey(100)
    
    for l in lines[0]:
        cv2.line(img, (l[0], l[1]), (l[2], l[3]), clr, 2)        
    
    cv2.imshow("rgb with all lines", img)
    cv2.waitKey(100)

    cut_line = vert_line[0]
     
    return cut_line


#########################################
### feature matching
#########################################
IROS_DATA_DIR = os.getenv("IROS_DATA_DIR")
task1 = 'InterruptedSuture5'
task2 = 'InterruptedSuture6'
pcf1 = osp.join(IROS_DATA_DIR, 'point_clouds', task1, 'pt0seg2_lh_rgb_pl.npy')
pcf2 = osp.join(IROS_DATA_DIR, 'point_clouds', task2, 'pt0seg2_lh_rgb_pl.npy')

xyz1 = osp.join(IROS_DATA_DIR, 'point_clouds', task1, 'pt0seg2_lh_xyz_tf.npy')
xyz2 = osp.join(IROS_DATA_DIR, 'point_clouds', task2, 'pt0seg2_lh_xyz_tf.npy')

kpf1 = osp.join(IROS_DATA_DIR, 'key_points', task1)
kpf2 = osp.join(IROS_DATA_DIR, 'key_points', task2)

#xyz1, rgb1 = load_xyzrgb("suture_scene0.pcd")
#xyz2, rgb2 = load_xyzrgb("suture_scene1.pcd")

rgb1 = np.load(pcf1)
rgb2 = np.load(pcf2)

kpts1 = np.load(kpf1 + '/pt0_keypoints.npy')
kpts2 = np.load(kpf2 + '/pt0_keypoints.npy')
kpts_names1 = np.load(kpf1 + '/pt0_keypoints_names.npy')
kpts_names2 = np.load(kpf2 + '/pt0_keypoints_names.npy')

for n in range(len(kpts_names1)):
    if kpts_names1[n] == ['left_hole', 'right_hole', 'cut']:
        kps1 = kpts1[n]
        break

for n in range(len(kpts_names2)):
    if kpts_names2[n] == ['left_hole', 'right_hole', 'cut']:
        kps2 = kpts2[n]
        break
        
print 'rgb1 kps', kps1
print 'rgb2 kps', kps2

#cv2.imshow("rgb1", rgb1)
#cv2.waitKey(100)

#cv2.imshow("rgb2", rgb2)
#cv2.waitKey(100)
