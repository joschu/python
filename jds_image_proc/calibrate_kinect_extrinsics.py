import numpy as np
import cv2
import calibration
import chessboards
from rave.openrave_kinematics import RaveRobot
from os.path import join
import jds_utils.transformations
import roslib;roslib.load_manifest('sensor_msgs')
import pcl_utils
import cPickle

#rr = RaveRobot()

DATA_DIR = "/home/joschu/Data/calib"

bgr_kinects = np.load(join(DATA_DIR,'bgr_kinects.npy'),mmap_mode='r')
bgr_lefts = np.load(join(DATA_DIR,'bgr_lefts.npy'),mmap_mode='r')

CB_SHAPE = (5,4)
CB_SIZE = .0249


def get_trans_rot_corners(bgr, window_name, cam_matrix):
    try:
        cam_trans, cam_rot, corners = chessboards.get_chessboard_pose(bgr, CB_SHAPE, CB_SIZE, cam_matrix)
        bgr_plot = bgr.copy()
        cv2.drawChessboardCorners(bgr_plot, CB_SHAPE, corners, True)
        cv2.circle(img=bgr_plot, center=tuple(corners[0].flatten()), radius=5, color=(0,0,255), thickness=2)
        cv2.imshow(window_name,bgr_plot)
        cv2.waitKey(20)
        print "chessboard found"
    except ValueError:
        cam_trans, cam_rot, corners = None,None, None            
        cv2.imshow(window_name,bgr.copy())
        cv2.waitKey(20)
        print "chessboard not found"
    return cam_trans, cam_rot, corners
        
cv2.namedWindow('kinect corners', cv2.cv.CV_WINDOW_NORMAL)
cv2.namedWindow('left corners', cv2.cv.CV_WINDOW_NORMAL)

kin_trans_list = []
cam_trans_list = []

with open('/home/joschu/Data/calib/info_left.pkl','r') as fh: cam_info=cPickle.load(fh)
left_cam_matrix = np.array(cam_info.P).reshape(3,4)[:3,:3]
kin_cam_matrix = pcl_utils.CAMERA_MATRIX

for (bgr_kin, bgr_left) in zip(bgr_kinects, bgr_lefts):
    kin_trans, kin_rot, kin_corn = get_trans_rot_corners(bgr_kin, 'kinect corners', kin_cam_matrix)
    cam_trans, cam_rot, cam_corn = get_trans_rot_corners(bgr_left, 'left corners', left_cam_matrix)
    if kin_trans is not None and cam_trans is not None:
        kin_trans_list.append(kin_trans.flatten())
        cam_trans_list.append(cam_trans.flatten())
        
        
M = transformations.superimposition_matrix(np.array(kin_trans_list),np.array(cam_trans_list))
