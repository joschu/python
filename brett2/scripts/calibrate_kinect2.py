from numpy import *
import jds_utils.transformations as trans
from traitsui.key_bindings import KeyBinding,KeyBindings
from traitsui.api import *
from traits.api import *
import numpy as np
from jds_utils.math_utils import *
import roslib
roslib.load_manifest("sensor_msgs")
roslib.load_manifest("rospy")
roslib.load_manifest("tf")
import sensor_msgs.msg as sm
import geometry_msgs.msg as gm
from jds_utils.conversions import trans_rot_to_pose
from jds_utils.transformations import quaternion_from_euler
import tf
from threading import Thread
import rospy

PARENT_FRAME = "/head_plate_frame"

def qm(a,b): return normalize(quaternion_multiply(a,b))

class Transform(HasTraits):
    t_step = .005
    
    R = Array(float,(3,))
    reset = Button
    T = Array(float,(3,))    
    
    _key_bindings = []
    

    for (ax,let,key) in zip('012','xyz','qwe'):
        for (sgn,suff) in zip(('1','-1'),('p','n')):
            if sgn == "-1": key = key.upper()
            fname = let+'q'+suff
                
            s = (
"""
def %(fname)s(self,_):
    t = r_[0.,0,0]
    t[%(ax)s] = %(sgn)s*self.t_step
    self.R = self.R + t
_key_bindings.append(KeyBinding(binding1="%(key)s",method_name="%(fname)s"))
"""%locals())
            print s
            exec(s)
            
            
    for (ax,let,key) in zip('012','xyz','asd'):
        for (sgn,suff) in zip(('1','-1'),('p','n')):
            if sgn == "-1": key = key.upper()
            fname = let+'q'+suff            
            fname = let+'t'+suff
            s = (
"""
def %(fname)s(self,_):
    t = r_[0.,0,0]
    t[%(ax)s] = %(sgn)s*self.t_step
    self.T = self.T + t

_key_bindings.append(KeyBinding(binding1="%(key)s",method_name="%(fname)s"))
"""%locals())
            print s
            exec(s)
            
    del ax,let,key,sgn,suff,fname

    def __init__(self,T=(0,0,0),R=(0,0,0)):
        self.T = T
        self.R = R
        HasTraits.__init__(self)

    def _reset_fired(self):
        #pvec =  self.arm.get_pvec(target_frame='wrist_roll_link')
        self.set(T=[0,0,0],R = [0,0,0], trait_change_notify=False)

    view = View(
        VGroup(
            Item("reset"),            
            Item("T"),
            Item("R")),
        key_bindings = KeyBindings(*_key_bindings),
        buttons = OKCancelButtons,
        width = 500)        




class TransformBroadcasterThread(Thread):
    wants_exit = False
    def __init__(self,transform):
        self.transform = transform
        self.pose_pub = rospy.Publisher("/kinect_pose", gm.PoseStamped)
        self.br = tf.TransformBroadcaster()
        Thread.__init__(self)
    def run(self):
        while not rospy.is_shutdown() and not self.wants_exit:
            rot = quaternion_from_euler(*self.transform.R)
            trans = self.transform.T
            self.br.sendTransform(trans, rot, rospy.Time.now(), "/camera2_link", PARENT_FRAME)
            ps = gm.PoseStamped()
            ps.pose = trans_rot_to_pose(trans,rot)
            ps.header.frame_id = PARENT_FRAME
            ps.header.stamp = rospy.Time.now()
            self.pose_pub.publish(ps)
            rospy.sleep(.01)
        
if __name__ == "__main__":

    rospy.init_node('calibration')
    tra = Transform()
    tf_thread = TransformBroadcasterThread(tra)
    tf_thread.start()
    tra.configure_traits()
    tf_thread.wants_exit = True
