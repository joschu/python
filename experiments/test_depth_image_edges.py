import rospy
import numpy as np, sensor_msgs.msg as sm
from brett2.ros_utils import image2numpy
import scipy.ndimage as ndi
import cv2
rospy.init_node("test_depth_image_edges")
msg = rospy.wait_for_message("/camera/depth_registered/image_rect", sm.Image)
msg1 = rospy.wait_for_message("/camera/rgb/image_rect_color", sm.Image)

depth = image2numpy(msg)
np.savetxt("/tmp/depth.txt",depth)

depth_fix_pos = depth.copy()
depth_fix_pos[np.isnan(depth)] = np.inf
depth_min = ndi.minimum_filter(depth_fix_pos, size=(3,3))

depth_fix_neg = depth.copy()
depth_fix_neg[np.isnan(depth)] = -np.inf
depth_max = ndi.maximum_filter(depth_fix_neg, size=(3,3))

max_minus_min = depth_max - depth_min
np.savetxt("/tmp/max_minus_min.txt",max_minus_min)

bgr = image2numpy(msg1)
cv2.imwrite("/tmp/bgr.jpg", bgr)
