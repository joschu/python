import rospy
from brett2 import ros_utils
from brett2.PR2 import PR2
from point_clouds import tabletop
import roslib;
roslib.load_manifest("verb_msgs")
from verb_msgs.srv import *
from brett2.ros_utils import xyzrgb2pc
import geometry_msgs.msg as gm
import sensor_msgs.msg as sm
import numpy as np
            
            
if __name__ == "__main__":
    if rospy.get_name() == "/unnamed": rospy.init_node("test_tabletop", disable_signals=True)
    pr2 = PR2.create()
    rviz = ros_utils.RvizWrapper.create()
    rospy.sleep(1)
    point_cloud = rospy.wait_for_message("/head_mount_kinect/depth_registered/points", sm.PointCloud2)     
    xyz, bgr = ros_utils.pc2xyzrgb(point_cloud)    
    xyz = xyz[200:, 200:500]
    xyz = ros_utils.transform_points(xyz, pr2.tf_listener, "base_footprint", point_cloud.header.frame_id)
    clusters = tabletop.segment_tabletop_scene(xyz,"base_footprint",plotting2d=True, plotting3d=True)
    
    if False:
        push_svc = rospy.ServiceProxy("push",Push)
        req = PushRequest()
        xyz0 = clusters[np.argmax(map(len,clusters))].reshape(1,-1,3)
        rgb0 = np.zeros(xyz0.shape,'uint8')
        point_cloud = xyzrgb2pc(xyz0, rgb0, "base_footprint")
        req.point_cloud = point_cloud
        push_svc.call(req)
        
    if False:
        pour_srv = rospy.ServiceProxy("pour", Pour)
        req = PourRequest()
        sort_inds = np.argsort(map(len,clusters))
        for i in sort_inds[:2]:
            xyz0 = clusters[i].reshape(1,-1,3)
            rgb0 = np.zeros(xyz0.shape,'uint8')
            cloud = xyzrgb2pc(xyz0, rgb0, "base_footprint")
            req.object_clouds.append(cloud)
        pour_srv.call(req)
    if True:
        env = pr2.robot.GetEnv()
        if env.GetViewer() is None: env.SetViewer('qtcoin')
        bounds = tabletop.get_table_dimensions(xyz)
        from kinematics import kinbodies
        kinbodies.create_box_from_bounds(env,bounds,"table")
        for (i_clu,clu) in enumerate(clusters):
            x,y,z,r,h = tabletop.get_cylinder(clu)
            print kinbodies.create_cylinder(env, (x,y,z),r,h, name="cylinder%i"%i_clu)
        