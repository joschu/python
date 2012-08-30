from jds_utils.conversions import trans_rot_to_hmat,hmat_to_trans_rot
from jds_utils.transformations import euler_from_quaternion
import rospy
import roslib
roslib.load_manifest("tf")
import tf
import numpy as np

class Globals:
    listener = None
    
if rospy.get_name() == "/unnamed":
    rospy.init_node("transform_kinect_frames")
    Globals.listener = tf.TransformListener()
    rospy.sleep(1)

ONIfromCL = trans_rot_to_hmat(*Globals.listener.lookupTransform("/camera_rgb_optical_frame","camera_link", rospy.Time(0)))
GAZfromONI = trans_rot_to_hmat([0, -.15, -.24], [0, 0, 0, 1])
HEADfromGAZ = trans_rot_to_hmat(*Globals.listener.lookupTransform("/head_plate_frame", "/wide_stereo_gazebo_l_stereo_camera_optical_frame",rospy.Time(0)))

HEADfromCL = np.dot(np.dot(HEADfromGAZ, GAZfromONI), ONIfromCL)

trans_final, quat_final = hmat_to_trans_rot(HEADfromCL)
eul_final = euler_from_quaternion(quat_final)

print "translation", trans_final
print "rotation", eul_final


print "%.4f %.4f %.4f %.4f %.4f %.4f %.4f"%(tuple(trans_final) + tuple(quat_final))