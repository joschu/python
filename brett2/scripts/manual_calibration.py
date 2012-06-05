from numpy import *
import transformations as trans
from traitsui.key_bindings import KeyBinding,KeyBindings
from traitsui.api import *
from traits.api import *
import numpy as np
from math_utils import *
import roslib
roslib.load_manifest("sensor_msgs")
roslib.load_manifest("rospy")
import sensor_msgs.msg as sm
from ros_utils import pc2xyzrgb, xyzrgb2pc

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

    def __init__(self,T,R):
        self.T = T
        self.R = R
        HasTraits.__init__(self)

    view = View(
        VGroup(
            Item("reset"),            
            Item("T",style="readonly"),
            Item("R",style="readonly")),
        key_bindings = KeyBindings(*_key_bindings),
        buttons = OKCancelButtons,
        width = 500)        


class CloudPlotter(Transform):        
    def __init__(self):
        self._reset_fired()
        Transform.__init__(self,self.T,self.R)
        
        self.sub = rospy.Subscriber("/camera/rgb/points", sm.PointCloud2, self.callback)
        self.pub = rospy.Publisher("/transformed_cloud", sm.PointCloud2)
        
        self.msg = None
        
    def _reset_fired(self):
        #pvec =  self.arm.get_pvec(target_frame='wrist_roll_link')
        self.set(T=[0,0,0],R = [0,0,0], trait_change_notify=False)
    
    def callback(self, msg):
        self.msg = msg
    
    @on_trait_change("T,R")
    def update_pose(self):
        
        print self.T, self.R
        
        if self.msg == None: return
        xyz, rgb = pc2xyzrgb(self.msg)
        xyz = xyz.reshape(-1,3)
        xyz1 = np.c_[xyz, np.ones((xyz.shape[0],1))]
        mat = trans.euler_matrix(*self.R)
        mat[:3,3] = self.T
        
        xyz1_transformed = np.dot(xyz1, mat.T)
        
        pc = xyzrgb2pc(xyz1_transformed[:,:3].reshape(480,640,3), rgb, "/wide_stereo_gazebo_l_stereo_camera_optical_frame")
        self.pub.publish(pc)
        
        
if __name__ == "__main__":

    import rospy
    rospy.init_node('manual_calibration')
    t = CloudPlotter()
    t.configure_traits()

