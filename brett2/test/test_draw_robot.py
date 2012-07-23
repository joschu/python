from brett2 import ros_utils
import geometry_msgs.msg as gm
import rospy

rospy.init_node("test_draw_robot")

rviz = ros_utils.RvizWrapper()
rospy.sleep(.5)

ps = gm.PoseStamped()
ps.header.frame_id = "base_link"
ps.header.stamp = rospy.Time.now()
handles = rviz.place_kin_tree_from_link(ps, "base_link")