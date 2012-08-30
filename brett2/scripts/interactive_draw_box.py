from traitsui.api import *
from traits.api import *
from jds_utils import conversions
from brett2.ros_utils import RvizWrapper,Marker
import geometry_msgs.msg as gm
import rospy
import numpy as np

class Box(HasTraits):
    cx = Range(-10.,10, 0) # center
    cy = Range(-10.,10, 0)
    cz = Range(-10.,10, 0)
    sx = Range(-3.,3, 1) # size
    sy = Range(-3.,3, 1)
    sz = Range(-3.,3, 1)
    theta = Range(0,2*np.pi)
    
    def __init__(self, rviz):
        HasTraits.__init__(self)
        self.rviz = rviz
        self.marker_handle = None
    
    @on_trait_change("cx,cy,cz,sx,sy,sz,theta")
    def draw(self):
        origin = [self.cx, self.cy, self.cz]
        quat = conversions.yaw_to_quat(self.theta)
        pose = conversions.trans_rot_to_pose(origin, quat)
        ps = gm.PoseStamped()
        ps.pose = pose
        ps.header.frame_id = "/base_link"
        ps.header.stamp = rospy.Time(0)
        print "(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), (%.3f, %.3f, %.3f)"%(
            self.cx, self.cy, self.cz, quat[0], quat[1], quat[2], quat[3], self.sx/2., self.sy/2., self.sz/2.)
        #rviz.draw_marker(ps, 0, type=Marker.CUBE, rgba=(0,1,0,.25), scale=(self.sx,self.sy,self.sz))
        self.marker_handle = rviz.place_kin_tree_from_link(ps, "base_link")
        
    view = View(
        VGroup(
            HGroup(Item("cx"), Item("cy"), Item("cz")),
            HGroup(Item("sx"), Item("sy"), Item("sz")),
            Item("theta")
        ),
        buttons = OKCancelButtons,
        width = 500,
        resizable = True)        
        
        
        
        
if __name__ == "__main__":
    rospy.init_node("draw_box",disable_signals = True)
    rviz = RvizWrapper()
    box = Box(rviz)
    box.configure_traits()