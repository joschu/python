from image_proc import pcd_io
import numpy as np
import cv2

comm_name = "pr2_rope1"

pcdfile = "/home/joschu/comm/%s/kinect/data000000000000.pcd"%comm_name
xyz,rgb = pcd_io.load_xyzrgb(pcdfile)
table_corners = np.loadtxt("/home/joschu/comm/%s/once/table_corners.txt"%comm_name)
x_ax = table_corners[1] - table_corners[0]
y_ax = table_corners[2]-table_corners[1]
z_ax = np.cross(x_ax, y_ax)

zimg = (z_ax[None,None,:] * xyz).sum(axis=2)

table_height = np.dot(z_ax, table_corners[0])

tablemask =  np.abs(zimg-table_height) <= .03

cv2.imshow('hi',(tablemask*100).astype('uint8'))
cv2.waitKey(10)