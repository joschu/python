from brett2 import ros_utils
import geometry_msgs.msg as gm
import rospy
import numpy as np

rospy.init_node("test_draw_gripper")

from jds_utils.conversions import array_to_pose_array
from brett2.ros_utils import RvizWrapper

rviz = RvizWrapper.create()

rospy.sleep(.5)



array = np.array([[0,0,0],[.2,0,0],[.4,0,0]])
pose_array = array_to_pose_array(array, 'base_footprint')
handles = rviz.draw_trajectory(pose_array, [0,.04,.08],'r')


