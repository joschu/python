#!/usr/bin/env python
"""
Deprecated
"""

from __future__ import division
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("file")
parser.add_argument("group")
parser.add_argument("--use_planner",action="store_true")
parser.add_argument("--test",action="store_true")
args = parser.parse_args()

import h5py
from brett2.PR2 import PR2,IKFail,transform_relative_pose_for_ik
from brett2 import trajectories
from kinematics import kinematics_utils, retiming
import rospy
from brett2.ros_utils import RvizWrapper,Marker, pc2xyzrgb, xyzrgb2pc
from numpy import asarray
import numpy as np
import geometry_msgs.msg as gm
from utils import conversions, math_utils, func_utils
from lfd import registration
import roslib; roslib.load_manifest("verb_msgs")
import itertools
from verb_msgs.srv import *
import lfd,os
from lfd.warping import draw_grid
from kinematics import kinbodies
from point_clouds import tabletop
import openravepy
import sensor_msgs.msg as sm
from brett2 import ros_utils
import traceback
if args.use_planner:
    roslib.load_manifest("pr2_control_utilities")
    from pr2_control_utilities import pr2_planner, pr2_joint_mover

HANDLES = []

def follow_rave_traj(arm, trajobj):
    n_wp = trajobj.GetNumWaypoints()
    arr = trajobj.GetWaypoints(0, n_wp).reshape(n_wp,16)
    velocities = arr[:,1:8]
    positions = arr[:,8:15]
    times = arr[:,0]
    arm.follow_timed_joint_trajectory(positions, velocities, times)

def create_obstacle_env(env):
    clone=env.CloneSelf(1)
    point_cloud = rospy.wait_for_message("/drop/points", sm.PointCloud2)     
    xyz, bgr = ros_utils.pc2xyzrgb(point_cloud)  
    xyz = ros_utils.transform_points(xyz, Globals.pr2.tf_listener, "base_footprint", point_cloud.header.frame_id)    
    #xyz = xyz[250:, 250:]
    clusters = tabletop.segment_tabletop_scene(xyz,'base_footprint')
    bounds = tabletop.get_table_dimensions(xyz)
    kinbodies.create_box_from_bounds(clone,bounds,"table")
    print len(clusters),"clusters"
    for (i_clu,clu) in enumerate(clusters):
        x,y,z,r,h = tabletop.get_cylinder(clu)
        #print kinbodies.create_cylinder(clone, (x,y,z),r,h, name="cylinder%i"%i_clu)
    return clone
        


@func_utils.once
def load_table():
    return True
    return Globals.pr2.robot.GetEnv().Load(os.path.join(os.path.dirname(lfd.__file__), "data", "table.xml"))
def draw_table():
    return
    aabb = Globals.pr2.robot.GetEnv().GetKinBody("table").GetLinks()[0].ComputeAABB()
    ps =gm.PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.pose.position = gm.Point(*aabb.pos())
    ps.pose.orientation = gm.Quaternion(0,0,0,1)
    HANDLES.append(Globals.rviz.draw_marker(ps, type=Marker.CUBE, scale = aabb.extents()*2, id = 24019,rgba = (1,0,0,.25)))

class Globals:
    pr2 = None
    rviz = None
    arm_mover = None
    
    def __init__(self): raise

    @staticmethod
    def setup():
        Globals.rviz = RvizWrapper.create()
        Globals.pr2 = PR2.create()
        rospy.sleep(.5)
        
COLORS = [(0,0,1,1), (0,1,0,1)]
def show_objects_and_trajectory(traj_xyz, obj_xyzs, obj_quats, obj_dims, rgba):
    for (i,obj_xyz, obj_quat,obj_dim) in zip(itertools.count(), obj_xyzs, obj_quats,obj_dims):
        ps = gm.PoseStamped(pose = gm.Pose(
            position = gm.Point(*obj_xyz),
            orientation = gm.Quaternion(*obj_quat)))
        ps.header.frame_id = 'base_footprint'
        HANDLES.append(Globals.rviz.draw_marker(
            ps,
            type=Marker.CUBE,
            rgba = rgba,
            scale = asarray(obj_dim)))
    pose_array = conversions.array_to_pose_array(asarray(traj_xyz), 'base_footprint')
    HANDLES.append(Globals.rviz.draw_curve(
        pose_array,
        rgba = rgba))




if __name__ == "__main__":
    if rospy.get_name() == "/unnamed": rospy.init_node("pour_service")
    Globals.setup()
    HANDLES = []

 

    traj = h5py.File(args.file, 'r')[args.group]
    
    decimation = 5
    old_gripper_quats = asarray(traj["r_gripper_orientations"])[::decimation]
    old_gripper_mats = conversions.quats2mats(old_gripper_quats)
    old_gripper_xyzs = asarray(traj["r_gripper_positions"])[::decimation]  + .18*old_gripper_mats[:,:,0]
    
    
    obj_dims = asarray(traj["object_dimension"][0])
    sort_inds = np.argsort(obj_dims.prod(axis=1))
    print sort_inds
    obj_dims = obj_dims[sort_inds]
    times = asarray(traj["time_stamp"])[::decimation]    
    times -= times[0]
    gripper_index = list(traj["joint_names"][0]).index("r_gripper_joint")
    gripper_angles = asarray(traj["joint_positions"][::decimation,gripper_index])
    gripper_angles[gripper_angles < .08] = 0
    gripper_angles[gripper_angles > .08] = .088

    
    f = registration.ThinPlateSpline()
    xyz1, xyz2 = asarray(traj["objects_position"][0])[sort_inds]
    quat1, quat2 = asarray(traj["objects_orientation"][0])[sort_inds]

    

    def callback(request):
        Globals.pr2.rarm.goto_posture('side')
        Globals.pr2.larm.goto_posture('side')

        
        
        #Globals.rviz.remove_all_markers()
        #draw_table()        
        new_cloud1, _ = pc2xyzrgb(request.object_clouds[0])
        new_cloud2, _ = pc2xyzrgb(request.object_clouds[1])
        new_cloud1, new_cloud2 = sorted([new_cloud1, new_cloud2], key = lambda x: np.squeeze(x).ptp(axis=0).prod())
        
        new_cloud1 = new_cloud1.reshape(-1,3)
        new_cloud2 = new_cloud2.reshape(-1,3)
        
        
        new_xyz1 = (new_cloud1.max(axis=0) + new_cloud1.min(axis=0))/2
        new_xyz2 = (new_cloud2.max(axis=0) + new_cloud2.min(axis=0))/2

        f.fit(np.array([xyz1, xyz2]), np.array([new_xyz1, new_xyz2]), 1e6, 1e-3)
    
        new_gripper_xyzs, new_gripper_mats = f.transform_frames(old_gripper_xyzs, conversions.quats2mats(old_gripper_quats))
        new_gripper_quats = conversions.mats2quats(new_gripper_mats)
        #print "warning: using old oreitnations"
    
        show_objects_and_trajectory(new_gripper_xyzs, np.array([new_xyz1, new_xyz2]), np.array([quat1, quat2]), obj_dims, (0,1,0,1))
        show_objects_and_trajectory(old_gripper_xyzs, np.array([xyz1, xyz2]), np.array([quat1, quat2]), obj_dims, (0,0,1,1))
        grid_handle = draw_grid(Globals.rviz, f.transform_points, old_gripper_xyzs.min(axis=0), old_gripper_xyzs.max(axis=0), "base_footprint")
        HANDLES.append(grid_handle)

        Globals.pr2.join_all()        
        Globals.pr2.update_rave()


        best_traj_info, best_feasible_frac = None, 0

        env = Globals.pr2.robot.GetEnv()
        Globals.pr2.update_rave()
        collision_env = create_obstacle_env(env)
        basemanip = openravepy.interfaces.BaseManipulation(collision_env.GetRobot("pr2"),plannername=None)
        rospy.sleep(.1)
        #collision_env.SetViewer("qtcoin")
        #raw_input("press enter to continue")

        for (lr, arm) in zip("lr",[Globals.pr2.larm,Globals.pr2.rarm]):
            name = arm.manip.GetName()
            manip = collision_env.GetRobot('pr2').GetManipulator(name)
            rospy.loginfo("trying to plan to initial position with %s",name)
            first_mat1 = np.eye(4)
            first_mat1[:3,:3] = new_gripper_mats[0]
            first_mat1[:3,3] = new_gripper_xyzs[0]
            first_mat = transform_relative_pose_for_ik(manip, first_mat1, "world", "%s_gripper_tool_frame"%lr)
            collision_env.GetRobot("pr2").SetActiveManipulator(name)
            trajobj = None
            try:
                trajobj = basemanip.MoveToHandPosition([first_mat],seedik=16,outputtrajobj=True,execute=0)
                rospy.loginfo("planning succeeded")
            except Exception:
                rospy.loginfo("planning failed")
                traceback.print_exc()
                print "initial ik result", manip.FindIKSolutions(first_mat,0)
                continue
            
            rospy.loginfo("trying ik")
            Globals.pr2.update_rave()
            joint_positions, inds = trajectories.make_joint_traj(new_gripper_xyzs, new_gripper_quats, manip, 'base_footprint', '%s_gripper_tool_frame'%lr, filter_options = 1+16)
            feasible_frac = len(inds)/len(new_gripper_xyzs)            
            print inds
            if feasible_frac > best_feasible_frac:
                best_feasible_frac = feasible_frac
                best_traj_info = dict(
                    feasible_frac = feasible_frac,
                    lr = 'l' if name == 'leftarm' else 'r',
                    manip = manip,
                    arm = arm,
                    initial_traj = trajobj,
                    joint_positions = joint_positions)
        
        collision_env.Destroy()
        response = PourResponse()
        
        if best_feasible_frac > .75:
            
            if best_traj_info["initial_traj"] is not None:
                follow_rave_traj(best_traj_info["arm"], best_traj_info["initial_traj"])
            else:
                print "no initial traj"
                #print "feasible inds", best_traj_info["inds"]
            
            body_traj = np.zeros(len(best_traj_info["joint_positions"]),dtype=trajectories.BodyState)
            lr = best_traj_info["lr"]
            body_traj["%s_arm"%lr] = best_traj_info["joint_positions"]
            body_traj["%s_gripper"%lr] = gripper_angles

            trajectories.follow_body_traj(Globals.pr2, body_traj, times, 
                    r_arm = (lr=='r'), r_gripper = (lr=='r'), l_arm = (lr=='l'), l_gripper= (lr=='l'))
            Globals.pr2.join_all()
            
            response.success = True
        else:
            rospy.logerr("could not execute trajectory because not enough points are reachable")
            response.success = False
        return response
        
        
    if args.test:
        
        from point_clouds import tabletop
        point_cloud = rospy.wait_for_message("/drop/points", sm.PointCloud2)     
        xyz, bgr = ros_utils.pc2xyzrgb(point_cloud)  
        xyz = ros_utils.transform_points(xyz, Globals.pr2.tf_listener, "base_footprint", point_cloud.header.frame_id)            
        #xyz = xyz[250:, 250:]
        clusters = tabletop.segment_tabletop_scene(xyz,plotting3d=1,plotting2d=0)
        clusters = sorted(clusters, key = len)
                
        
        req = PourRequest()
        _xyz0 = np.array([clusters[0]])
        _xyz1 = np.array([clusters[1]])
        _rgb0 = np.zeros(_xyz0.shape,'uint8')
        _rgb1 = np.zeros(_xyz1.shape,'uint8')
        req.object_clouds.append(xyzrgb2pc(_xyz0, _rgb0,'base_footprint'))
        req.object_clouds.append(xyzrgb2pc(_xyz1, _rgb1,'base_footprint'))

        success = callback(req)
        print success
    else:
        rospy.Service("pour", Pour, callback)
        print "ready"
        rospy.spin()
