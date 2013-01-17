#!/usr/bin/env python

import roslib; roslib.load_manifest("smach_ros")
from kinematics import kinbodies
import geometry_msgs.msg as gm
import rospy
from brett2 import ros_utils, PR2
from brett2.ros_utils import Marker

def draw_table():
    aabb = Globals.pr2.robot.GetEnv().GetKinBody("table").GetLinks()[0].ComputeAABB()
    ps =gm.PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.pose.position = gm.Point(*aabb.pos())
    ps.pose.orientation = gm.Quaternion(0,0,0,1)
    Globals.handles.append(Globals.rviz.draw_marker(ps, type=Marker.CUBE, scale = aabb.extents()*2, id = 24019,rgba = (1,0,0,.25)))

def load_table():
    table_bounds = map(float, rospy.get_param("table_bounds").split())
    kinbodies.create_box_from_bounds(Globals.pr2.env,table_bounds, name="table")

class Globals:
    pr2 = None
    rviz = None
    handles = []
    isinstance(pr2, PR2.PR2)
    isinstance(rviz, ros_utils.RvizWrapper)
  
    def __init__(self): raise

    @staticmethod
    def setup():
        if Globals.pr2 is None: 
            Globals.pr2 = PR2.PR2.create()
            load_table()
        if Globals.rviz is None: Globals.rviz = ros_utils.RvizWrapper.create()


if __name__ == '__main__':
    Globals.handles = []
    if rospy.get_name() == '/unnamed':
    	rospy.init_node("draw_table", disable_signals=True)
    Globals.setup()
    draw_table()
    raw_input('press enter to exit')
