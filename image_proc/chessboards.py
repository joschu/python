from utils import conversions
import cv2
import numpy as np


def get_chessboard_pose(im,shape_cb,size,cam_matrix):
    width_cb,height_cb = shape_cb
    xs,ys = np.meshgrid(range(0,width_cb),range(0,height_cb))
    xys = size*np.c_[xs.flat,ys.flat,np.zeros(xs.size)]
    print xys

    height_im,width_im = im.shape[:2]
    cx = width_im/2-.5
    cy = height_im/2-.5

    dist_coeffs = np.zeros(5)
    corners = cv2.findChessboardCorners(im,(width_cb,height_cb))[1]
    if corners is None:
        raise ValueError
    else:        
        rod,trans = cv2.solvePnP(np.array(xys),np.array(corners),cam_matrix,dist_coeffs)
        quat = conversions.mat2quat(conversions.rod2mat(rod))
        return trans,quat,corners
