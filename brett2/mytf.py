"""
Fuck tf
"""

import rospy
import networkx as nx
from jds_utils import conversions
from jds_utils.func_utils import memoized
import numpy as np
from numpy.linalg import inv
import roslib;
roslib.load_manifest("tf")
import geometry_msgs.msg as gm
import tf

class TransformListener(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/tf",tf.msg.tfMessage,self.callback)
        self.transformer = MyTransformer()
    def callback(self, msg):
        for tstamped in msg.transforms:
            self.transformer.add_transform(tstamped.header.frame_id, tstamped.child_frame_id, tstamped.transform)
    def lookupTransform(self,to_frame, from_frame, *args):
        if not to_frame.startswith('/'): to_frame = '/'+to_frame
        if not from_frame.startswith('/'): to_frame = '/'+from_frame
        hmat = self.transformer.lookup_transform_mat(to_frame, from_frame)
        return conversions.hmat_to_trans_rot(hmat)

class MyTransformer(object):
    def __init__(self):
        self.G = nx.DiGraph()

    def add_transform(self, to_frame, from_frame, trans_rot):
        if not self.G.has_edge(to_frame, from_frame):
            if not self.G.has_node(to_frame):
                self.G.add_node(to_frame)
            if not self.G.has_node(from_frame):
                self.G.add_node(from_frame)
            self.G.add_edge(to_frame, from_frame)
            self.G.has_edge(from_frame, to_frame)
            self.G.add_edge(from_frame, to_frame)
        self.G.edge[to_frame][from_frame]["transform"] = trans_rot
    
    def lookup_transform_mat(self, to_frame, from_frame):
        path = self.get_path(to_frame,from_frame)
        hmat = np.eye(4)
        for (n0, n1) in zip(path[:-1], path[1:]):
            if "transform" in self.G.edge[n0][n1]:
                hmat1 = ros_transform_to_hmat(self.G.edge[n0][n1]["transform"])
            else:
                hmat1 = inv(ros_transform_to_hmat(self.G.edge[n1][n0]["transform"]))
            hmat = np.dot(hmat, hmat1)
        return hmat
    
    @memoized
    def get_path(self, to_frame, from_frame):
        return nx.shortest_path(self.G, to_frame, from_frame)
        
def ros_transform_to_hmat(tros):
    return conversions.trans_rot_to_hmat(
        [tros.translation.x, tros.translation.y, tros.translation.z],
        [tros.rotation.x, tros.rotation.y, tros.rotation.z, tros.rotation.w])
def hmat_to_ros_transform(hmat):
    (x,y,z), (xx,yy,zz,ww) = conversions.hmat_to_trans_rot(hmat)
    return gm.Transform(translation = gm.Vector3(x,y,z),
                        rotation = gm.Quaternion(xx,yy,zz,ww))










        
def test1():
    "check to see that transform is correct"
    import openravepy
    env = openravepy.Environment()
    env.Load('robots/pr2-beta-static.zae')
    pr2 = env.GetRobot('pr2')
    mytf = MyTransformer()
    i_base = pr2.GetLink("base_footprint").GetIndex()
    i_rgrip =  pr2.GetLink("r_gripper_tool_frame").GetIndex()
    chain = pr2.GetChain(i_base, i_rgrip)
    dofvalues = pr2.GetDOFValues()
    pr2.SetDOFValues(dofvalues + .1*np.random.randn(dofvalues.size))
    for joint in chain:
        parent = joint.GetHierarchyParentLink()
        child = joint.GetHierarchyChildLink()
        mytf.add_transform('/'+child.GetName(), '/'+parent.GetName(), hmat_to_ros_transform(np.dot(inv(child.GetTransform()), parent.GetTransform())))
    print mytf.lookup_transform_mat('/base_footprint','/r_wrist_roll_link')
    print mytf.lookup_transform_mat('/r_wrist_roll_link','/base_footprint')
    print pr2.GetLink("r_wrist_roll_link").GetTransform()
    
def test2():
    rospy.init_node('test_mytf')
    mytf = TransformListener()
    rospy.sleep(10)
    print mytf.lookupTransform("/base_link","/base_footprint")
    print mytf.lookupTransform("/r_gripper_tool_frame","/base_link")
    rospy.spin()

if __name__ == "__main__":
    test1()
    test2()