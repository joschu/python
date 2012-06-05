from image_proc.pcd_io import load_xyzrgb
import rope_vision.rope_initialization as ri
import numpy as np
import sys
import networkx as nx
import itertools
from mayavi import mlab
from brett2 import ros_utils
import sensor_msgs.msg as sm
            
    
import rospy
rospy.init_node("test_rope_init")
cloud = rospy.wait_for_message("/preprocessor/points", sm.PointCloud2)
xyz,rgb = ros_utils.pc2xyzrgb(cloud)
# xyz, rgb = load_xyzrgb("data000000000004.pcd")
# xyz = xyz.reshape(-1,3)
#xyz = np.squeeze(np.loadtxt("/tmp/rope.txt"))
total_path_3d = ri.find_path_through_point_cloud(xyz, plotting=True)
