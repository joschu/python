#!/usr/bin/env python

"""
Perform deformable object manipulation task, where data is stored in some h5 file
Currently works for tying an overhand knot or folding up a laid-out towel
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--obj", choices=["rope","cloth"])
args = parser.parse_args()

if args.obj == "rope":
    DS_METHOD = "voxel"
    DS_LENGTH = .025
    H5FILE = "knot_segments.h5"
    HEAD_TILT=.9
if args.obj == "cloth":
    DS_METHOD = "hull"
    DS_LENGTH = .025
    H5FILE = "fold_segments.h5"
    HEAD_TILT=1.1

import roslib; roslib.load_manifest("smach_ros")
import smach
from lfd import registration, trajectory_library, warping, recognition
import lfd
from kinematics import reachability
from jds_utils.yes_or_no import yes_or_no
import sensor_msgs.msg
import geometry_msgs.msg as gm
import rospy
import os
from jds_image_proc import curves, interactive_roi as roi
from brett2 import ros_utils, PR2, trajectories as trajectories
from brett2.ros_utils import Marker
import numpy as np
from jds_utils import conversions
from knot_tying import tps
from jds_image_proc.clouds import voxel_downsample
from jds_image_proc.alpha_shapes import get_concave_hull

RVIZ_LPOS = 0
RVIZ_RPOS = 1

HUMAN_SELECT_DEMO=False
HUMAN_GET_ROPE=False

    

def human_get_rope():
    point_cloud = rospy.wait_for_message("/drop/points", sensor_msgs.msg.PointCloud2)
    
    xyz, bgr = ros_utils.pc2xyzrgb(point_cloud)
    
    xys = roi.get_polyline(bgr,'draw curve')
    uvs = np.int32(xys)[:,::-1]
    us,vs = uvs.T
    xyzs = xyz[us,vs]
    xyzs_good = xyzs[np.isfinite(xyzs).all(axis=1)]
    print "%i out of %i labeled points are good"%(len(xyzs_good), len(xyzs))
    xyzs_unif = curves.unif_resample(xyzs_good,100,tol=.002)
    return xyzs_unif, point_cloud.header.frame_id
def draw_table():
    aabb = Globals.pr2.robot.GetEnv().GetKinBody("table").GetLinks()[0].ComputeAABB()
    ps =gm.PoseStamped()
    ps.header.frame_id = "base_footprint"
    ps.pose.position = gm.Point(*aabb.pos())
    ps.pose.orientation = gm.Quaternion(0,0,0,1)
    HANDLES.append(Globals.rviz.draw_marker(ps, type=Marker.CUBE, scale = aabb.extents()*2, id = 24019,rgba = (1,0,0,.25)))

                      
    
def fix_gripper(g):
    g = np.asarray(g).copy()
    g[g < .015] = 0
    #g[g >= .04] = .08
    return g

class Globals:
    pr2 = None
    rviz = None
    isinstance(pr2, PR2.PR2)
    isinstance(rviz, ros_utils.RvizWrapper)
    
    def __init__(self): raise

    @staticmethod
    def setup():
        if Globals.pr2 is None: 
            Globals.pr2 = PR2.PR2.create()
            Globals.pr2.robot.GetEnv().Load(os.path.join(os.path.dirname(lfd.__file__), "data", "table.xml"))
        if Globals.rviz is None: Globals.rviz = ros_utils.RvizWrapper.create()

class LookAtObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes = ["success", "failure"],
            input_keys = [],
            output_keys = ["points"] # object points
        )
            
        
    def execute(self,userdata):
        """
        - move head to the right place
        - get a point cloud
        returns: success, failure
        """
        global HANDLES
        HANDLES=[]
        
        # Globals.pr2.head.set_pan_tilt(0, HEAD_TILT)
        Globals.pr2.larm.goto_posture('side')
        Globals.pr2.rarm.goto_posture('side')
        Globals.pr2.join_all()
        if HUMAN_GET_ROPE:
            xyz,frame = human_get_rope()
            xyz = ros_utils.transform_points(xyz, Globals.pr2.tf_listener, "base_footprint", frame)
            pose_array = conversions.array_to_pose_array(xyz,"base_footprint")
            HANDLES.append(Globals.rviz.draw_curve(pose_array,id=3250864,rgba=(0,0,1,1)))
            xyz = curves.unif_resample(xyz, 100,tol=.01)

        else:
            msg = rospy.wait_for_message("/preprocessor/points", sensor_msgs.msg.PointCloud2)
            xyz, rgb = ros_utils.pc2xyzrgb(msg)
            xyz = xyz.reshape(-1,3)
            xyz = ros_utils.transform_points(xyz, Globals.pr2.tf_listener, "base_footprint", msg.header.frame_id)
        userdata.points = xyz
                
        return "success"
    
def downsample(xyz):
    if DS_METHOD == "voxel":        
        xyz_ds, ds_inds = voxel_downsample(xyz, DS_LENGTH, return_inds = True)
    elif DS_METHOD == "hull":
        xyz = np.squeeze(xyz)
        _, inds = get_concave_hull(xyz[:,0:2],.05)
        xyz_ds = xyz[inds]
        ds_inds = [[i] for i in inds]    
    return xyz_ds, ds_inds
    
        
def alternate(arr1, arr2):
    assert arr1.shape == arr2.shape
    out = np.zeros((2*arr1.shape[0], arr1.shape[1]),arr1.dtype)
    out[0::2] = arr1
    out[1::2] = arr2
    return out
        
class SelectTrajectory(smach.State):
    f = None
    def __init__(self):
        smach.State.__init__(self, 
            outcomes = ["done", "not_done","failure"],
            input_keys = ["points"],
            output_keys = ["trajectory", "left_used", "right_used"])
            
        self.library = trajectory_library.TrajectoryLibrary(H5FILE, "read")
        self.db = reachability.ReachabilityDatabase("read");

    def execute(self,userdata):
        """
        - lookup closest trajectory from database
        - if it's a terminal state, we're done
        - warp it based on the current rope
        returns: done, not_done, failure
        """
        xyz_new = np.squeeze(np.asarray(userdata.points))
        if args.obj == "cloth": xyz_new = voxel_downsample(xyz_new, .025)
        
        xyz_new_ds, ds_inds = downsample(xyz_new)
        dists_new = recognition.calc_geodesic_distances_downsampled(xyz_new,xyz_new_ds, ds_inds)
        
        if HUMAN_SELECT_DEMO:
            seg_name = trajectory_library.interactive_select_demo(self.library)
            best_demo = self.library.root[seg_name]         
            pts0,_ = downsample(np.asarray(best_demo["cloud_xyz"]))
            pts1,_ = downsample(xyz_new)
            self.f = registration.tps_rpm(pts0, pts1, 
                                     plotting = 4, reg_init=1,reg_final=.025,n_iter=40)                
        else:
            
            best_f = None
            best_cost = np.inf
            best_name = None
            for (seg_name,candidate_demo) in self.library.root.items():
                xyz_demo = np.squeeze(np.asarray(candidate_demo["cloud_xyz"]))
                if args.obj == "cloth": xyz_demo = voxel_downsample(xyz_demo, .025)
                xyz_demo_ds, ds_inds = downsample(xyz_demo)#voxel_downsample(xyz_demo, DS_LENGTH, return_inds = True)
                dists_demo = recognition.calc_geodesic_distances_downsampled(xyz_demo, xyz_demo_ds, ds_inds)
                cost = recognition.calc_match_score(xyz_new_ds, xyz_demo_ds, dists0 = dists_new, dists1 = dists_demo)
                print "seg_name: %s. cost: %s"%(seg_name, cost)
                if cost < best_cost:
                    best_cost = cost
                    best_name = seg_name

            #if best_name.startswith("done"): return "done"
            best_demo = self.library.root[best_name]
            xyz_demo_ds,_ = downsample(np.asarray(best_demo["cloud_xyz"][0]))
            self.f = registration.tps_rpm(xyz_demo_ds, xyz_new_ds, 
                            plotting = 10, reg_init=1,reg_final=.01,n_iter=200,verbose=True)                

            print "best segment:", best_name

        

        orig_pose_array = conversions.array_to_pose_array(best_demo["cloud_xyz"][0], "base_footprint")
        warped_pose_array = conversions.array_to_pose_array(self.f.transform_points(best_demo["cloud_xyz"][0]), "base_footprint")
        HANDLES.append(Globals.rviz.draw_curve(orig_pose_array,rgba=(1,0,0,1),id=19024,type=Marker.CUBE_LIST))
        HANDLES.append(Globals.rviz.draw_curve(warped_pose_array,rgba=(0,1,0,1),id=2983,type=Marker.CUBE_LIST))

        mins = best_demo["cloud_xyz"][0].min(axis=0)
        maxes = best_demo["cloud_xyz"][0].max(axis=0)
        mins[2] -= .1
        maxes[2] += .1
        grid_handle = warping.draw_grid(Globals.rviz, self.f.transform_points, mins, maxes, 'base_footprint')
        HANDLES.append(grid_handle)
        
        userdata.left_used = left_used = best_demo["arms_used"].value in "lb"
        userdata.right_used = right_used = best_demo["arms_used"].value in "rb"
        print "left_used", left_used
        print "right_used", right_used
        
        warped_demo = warping.transform_demo_with_fingertips(self.f, best_demo, left=left_used, right=right_used)
        trajectory = np.zeros(len(best_demo["times"]), dtype=trajectories.BodyState)                        
        
        Globals.pr2.update_rave()          
        if left_used:            
            l_arm_traj, feas_inds = trajectories.make_joint_traj(warped_demo["l_gripper_xyzs"], warped_demo["l_gripper_quats"], Globals.pr2.robot.GetManipulator("leftarm"),"base_footprint","l_gripper_tool_frame",1+16)            
            if len(feas_inds) == 0: return "failure"
            trajectory["l_arm"] = l_arm_traj
            rospy.loginfo("left arm: %i of %i points feasible", len(feas_inds), len(trajectory))
            trajectory["l_gripper"] = fix_gripper(warped_demo["l_gripper_angles"])
            HANDLES.append(Globals.rviz.draw_curve(
                conversions.array_to_pose_array(
                    alternate(warped_demo["l_gripper_xyzs1"],warped_demo["l_gripper_xyzs2"]), 
                    "base_footprint"), 
                width=.001, rgba = (1,0,1,.4),type=Marker.LINE_LIST))
        if right_used:
            r_arm_traj,feas_inds = trajectories.make_joint_traj(warped_demo["r_gripper_xyzs"], warped_demo["r_gripper_quats"], Globals.pr2.robot.GetManipulator("rightarm"),"base_footprint","r_gripper_tool_frame",1+16)            
            if len(feas_inds) == 0: return "failure"
            trajectory["r_arm"] = r_arm_traj
            rospy.loginfo("right arm: %i of %i points feasible", len(feas_inds), len(trajectory))            
            trajectory["r_gripper"] = fix_gripper(warped_demo["r_gripper_angles"])
            HANDLES.append(Globals.rviz.draw_curve(
                conversions.array_to_pose_array(
                    alternate(warped_demo["l_gripper_xyzs1"],warped_demo["l_gripper_xyzs2"]), 
                    "base_footprint"), 
                width=.001, rgba = (1,0,1,.4),type=Marker.LINE_LIST))
        userdata.trajectory = trajectory
        #userdata.base_xya = (x,y,0)
        # todo: draw pr2        
        # consent = yes_or_no("trajectory ok?")
        consent = True
        if consent: return "not_done"
        else: return "failure"
            
        
                
class MoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes = ["success", "failure"],
            input_keys = ["base_ps"],
            output_keys = [])
            

    def execute(self, userdata):
        """
        - move base to a target position
        returns: success, failure
        
        visualization:
        - publish base pose in rviz
        """
        x,y,a = userdata.base_xya
        Globals.pr2.base.goto_pose(x,y,a,"table_home") # xxx 
        
class AdjustTrajectory(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes = ["success", "failure"],
            input_keys = [],
            output_keys = [])
        
    def execute(self, userdata):
        """
        - move head to look at object again
        - do icp and compare to the last time you saw it
        - adjust poses of trajectory
        - generate actuall joint trajectory
        returns: success, failure
        
        currently does nothing because I don't know if it's necessary
        
        """
        return "success"

class ExecuteTrajectory(smach.State):
    def __init__(self):
        """
        - first show trajectory in rviz and see if it's ok
        - then execute it
        returns: success, failure
        """
        smach.State.__init__(self, 
            outcomes = ["success", "failure"],
            input_keys = ["trajectory","left_used","right_used"],
            output_keys = [])
            

    def execute(self, userdata):
        Globals.pr2.update_rave()
        trajectories.follow_body_traj(Globals.pr2, userdata.trajectory, times=None,
            l_arm = userdata.left_used, r_arm = userdata.right_used,
            l_gripper = userdata.left_used, r_gripper = userdata.right_used)
        return "success"
        
def make_tie_knot_sm():
    sm = smach.StateMachine(outcomes = ["success", "failure"])
    with sm:
        smach.StateMachine.add("look_at_object", LookAtObject(), transitions = {"success":"select_traj", "failure":"failure"})
        smach.StateMachine.add("select_traj", SelectTrajectory(), transitions = {"done":"success","not_done":"execute_traj", "failure":"failure"})
        #smach.StateMachine.add("move_base", MoveBase(), transitions = {"success":"adjust_traj","failure":"failure"})
        #smach.StateMachine.add("adjust_traj", AdjustTrajectory(), transitions = {"success":"execute_traj","failure":"failure"})
        smach.StateMachine.add("execute_traj", ExecuteTrajectory(), transitions = {"success":"look_at_object","failure":"failure"})
        
        
    return sm

if __name__ == "__main__":
    HANDLES = []
    if rospy.get_name() == '/unnamed':
        rospy.init_node("tie_knot",disable_signals=True)
    Globals.setup()
    draw_table()
    #Globals.pr2.torso.go_up()
    Globals.pr2.join_all()
    tie_knot_sm = make_tie_knot_sm()
    tie_knot_sm.execute()
