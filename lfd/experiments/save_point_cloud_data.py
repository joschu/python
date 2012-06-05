from brett2 import ros_utils
import roslib;
roslib.load_manifest('tf')
import tf
import rospy
import sensor_msgs.msg as sm
import numpy as np
from utils import conversions

rospy.init_node("save_point_cloud_data")
listener = tf.TransformListener()
rospy.sleep(.5)



xyzs = []
while True:
    response = raw_input("press enter when you're ready. type 'quit' when done")
    if 'quit' in response: break
    pc = rospy.wait_for_message("/preprocessor/points", sm.PointCloud2)
    trans,rot=listener.lookupTransform("base_footprint",pc.header.frame_id, rospy.Time(0))
    hmat = conversions.trans_rot_to_hmat(trans,rot)
    xyz, rgb = ros_utils.pc2xyzrgb(pc)
    xyz = np.squeeze(xyz)
    xyz1 = np.c_[xyz, np.ones((len(xyz),1))]
    xyz1_base = np.dot(xyz1, hmat.T)
    xyzs.append(xyz1_base[:,:3])
    
print "saving"
np.save("/home/joschu/Data/rope/ropes.npy",np.array(xyzs,dtype=object))
