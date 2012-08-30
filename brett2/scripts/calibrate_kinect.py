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
import sensor_msgs.msg as sm
import geometry_msgs.msg as gm
from jds_utils.conversions import hmat_to_pose

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
            Item("T"),
            Item("R")),
        key_bindings = KeyBindings(*_key_bindings),
        buttons = OKCancelButtons,
        width = 500)        


def pc2xyzrgb(pc):
    arr = np.fromstring(pc.data,dtype='float32').reshape(pc.height,pc.width,8)
    xyz = arr[:,:,0:3]
    
    rgb0 = np.ndarray(buffer=arr[:,:,4].copy(),shape=(480,640,4),dtype='uint8')
    rgb = rgb0[:,:,0:3]
    
    return xyz,rgb
def xyzrgb2pc(xyz,bgr, frame_id):
    arr = np.empty((480,640,8),dtype='float32')
    arr[:,:,0:3] = xyz
    bgr1 = np.empty((480,640,4),dtype='uint8')
    bgr1[:,:,0:3] = bgr    
    arr[:,:,4] = bgr1.view(dtype='float32').reshape(480,640)
    data = arr.tostring()
    msg = sm.PointCloud2()
    msg.data = data
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.fields = [sm.PointField(name='x',offset=0,datatype=7,count=1),
                  sm.PointField(name='y',offset=4,datatype=7,count=1),
                  sm.PointField(name='z',offset=8,datatype=7,count=1),
                  sm.PointField(name='rgb',offset=16,datatype=7,count=1)]
    msg.is_dense = False
    msg.width=640
    msg.height=480
    msg.header.stamp = rospy.Time.now()
    msg.point_step = 32
    msg.row_step = 20480
    msg.is_bigendian = False
    
    return msg


class Teleop(Transform):        
    def __init__(self):
        self._reset_fired()
        Transform.__init__(self,self.T,self.R)
        
        self.sub = rospy.Subscriber("/camera/depth_registered/points", sm.PointCloud2, self.callback)
        self.pub = rospy.Publisher("/transformed_cloud", sm.PointCloud2)
        self.pose_pub = rospy.Publisher("/kinect_pose", gm.PoseStamped)
        self.msg = None
        
    def _reset_fired(self):
        #pvec =  self.arm.get_pvec(target_frame='wrist_roll_link')
        self.set(T=[0,0,0],R = [0,0,0], trait_change_notify=False)
    
    def callback(self, msg):
        self.msg = msg
    
    @on_trait_change("T,R")
    def update_pose(self):
        
        mat = trans.euler_matrix(*self.R)
        mat[:3,3] = self.T
        print self.T, self.R

        ps = gm.PoseStamped()
        ps.pose = hmat_to_pose(mat)
        ps.header.frame_id =  "/wide_stereo_gazebo_l_stereo_camera_optical_frame"
        self.pose_pub.publish(ps)
        
        if self.msg == None: return
        xyz, rgb = pc2xyzrgb(self.msg)
        xyz = xyz.reshape(-1,3)
        xyz1 = np.c_[xyz, np.ones((xyz.shape[0],1))]

        
        xyz1_transformed = np.dot(xyz1, mat.T)
        
        pc = xyzrgb2pc(xyz1_transformed[:,:3].reshape(480,640,3), rgb, "/wide_stereo_gazebo_l_stereo_camera_optical_frame")
        self.pub.publish(pc)


        
        
if __name__ == "__main__":

    import rospy
    rospy.init_node('blha')
    t = Teleop()
    t.configure_traits()
