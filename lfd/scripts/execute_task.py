#!/usr/bin/env python

"""
Perform deformable object manipulation task, where data is stored in some h5 file
Currently works for tying an overhand knot or folding up a laid-out towel
"""

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--task")
parser.add_argument("--human_select_demo", action="store_true")
parser.add_argument("--prompt_before_motion", action="store_true")
parser.add_argument("--count_steps",action="store_true")
parser.add_argument("--hard_table",action="store_true")
parser.add_argument("--test",action="store_true")
parser.add_argument("--use_tracking", action="store_true")
parser.add_argument("--reg_final", type=float, default=.025)
parser.add_argument("--use_rigid", action="store_true")
parser.add_argument("--cloud_topic", type=str, default="/preprocessor/points")
args = parser.parse_args()


import roslib; roslib.load_manifest("smach_ros")
import smach
import lfd
from lfd import registration, trajectory_library, warping, recognition, lfd_traj
from kinematics import kinbodies
from jds_utils.yes_or_no import yes_or_no
import sensor_msgs.msg
import geometry_msgs.msg as gm
import rospy
import os
import os.path as osp
from brett2 import ros_utils, PR2
from brett2.ros_utils import Marker
import numpy as np
from jds_utils import conversions
from jds_image_proc.clouds import voxel_downsample
try:
    from jds_image_proc.alpha_shapes import get_concave_hull
except Exception:
    pass
try:
    from bulletsim_msgs.msg import TrackedObject
except Exception:
    pass
import h5py
from collections import defaultdict
import yaml
    
data_dir = osp.join(osp.dirname(lfd.__file__), "data")
with open(osp.join(data_dir, "knot_demos.yaml"),"r") as fh: 
    task_info = yaml.load(fh)
    
DS_LENGTH = .025
DS_METHOD = "voxel"
if args.task.startswith("fold"):
    DS_METHOD="hull"
#else:
    #DS_METHOD = "voxel"
    
H5FILE = osp.join(data_dir, task_info[args.task]["db_file"])
demos_file = h5py.File(H5FILE,"r")
rospy.loginfo("loading demos into memory")
demos = warping.group_to_dict(demos_file)    
    
if args.test:
    lfd_traj.ALWAYS_FAKE_SUCCESS = True
    
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
    
def increment_pose(arm, translation):
    cur_pose = arm.get_pose_matrix("base_footprint", "r_gripper_tool_frame")
    new_pose = cur_pose.copy()
    new_pose[:3,3] += translation
    arm.goto_pose_matrix(new_pose, "base_footprint", "r_gripper_tool_frame")
        
    

class Globals:
    pr2 = None
    rviz = None
    handles = []
    isinstance(pr2, PR2.PR2)
    isinstance(rviz, ros_utils.RvizWrapper)
    if args.count_steps: stage = 0
    
    def __init__(self): raise

    @staticmethod
    def setup():
        if Globals.pr2 is None: 
            Globals.pr2 = PR2.PR2.create(rave_only=args.test)
            if not args.test: load_table()
        if Globals.rviz is None: Globals.rviz = ros_utils.RvizWrapper.create()
        Globals.table_height = rospy.get_param("table_height")


def select_from_list(list):
    strlist = [str(item) for item in list]
    while True:
        print "choose from the following options:"
        print " ".join("(%s)"%item for item in strlist)
        resp = raw_input("?) ")
        if resp not in strlist:
            print "invalid response. try again."
        else:
            return list[strlist.index(resp)]            

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
        Globals.handles = []
        draw_table()
        
        Globals.pr2.rgrip.set_angle(.08)
        Globals.pr2.lgrip.set_angle(.08)
        Globals.pr2.join_all()
        if not args.use_tracking:
            Globals.pr2.larm.goto_posture('side')
            Globals.pr2.rarm.goto_posture('side')
        else:
            try: increment_pose(Globals.pr2.rarm, [0,0,.02])
            except PR2.IKFail: print "couldn't raise right arm"
            try: increment_pose(Globals.pr2.larm, [0,0,.02])
            except PR2.IKFail: print "couldn't raise left arm"
        Globals.pr2.rgrip.set_angle(.08)
        Globals.pr2.lgrip.set_angle(.08)
        Globals.pr2.join_all()


        if args.test:
            xyz = np.squeeze(np.asarray(demos[select_from_list(demos.keys())]["cloud_xyz"]))
        elif args.use_tracking:
            msg = rospy.wait_for_message("/tracker/object", TrackedObject)
            xyz = [(pt.x, pt.y, pt.z) for pt in msg.rope.nodes]
        else:
            msg = rospy.wait_for_message(args.cloud_topic, sensor_msgs.msg.PointCloud2)
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
    

def calc_seg_cost(seg_name, xyz_new_ds, dists_new):
    candidate_demo = demos[seg_name]
    xyz_demo_ds = np.squeeze(candidate_demo["cloud_xyz_ds"])
    dists_demo = candidate_demo["geodesic_dists"]
  #  cost = recognition.calc_match_score(xyz_new_ds, xyz_demo_ds, dists0 = dists_new, dists1 = dists_demo)
    cost = recognition.calc_match_score(xyz_demo_ds, xyz_new_ds, dists0 = dists_demo, dists1 = dists_new)
    print "seg_name: %s. cost: %s"%(seg_name, cost)
    return cost, seg_name

    
        
class SelectTrajectory(smach.State):
    f = None
    def __init__(self):
        smach.State.__init__(self, 
            outcomes = ["done", "not_done","failure"],
            input_keys = ["points"],
            output_keys = ["trajectory"])
        

        
        rospy.loginfo("preprocessing demo point clouds...")
        for (_,demo) in demos.items():
            demo["cloud_xyz_ds"], ds_inds = downsample(demo["cloud_xyz"])
            demo["cloud_xyz"] = np.squeeze(demo["cloud_xyz"])
            demo["geodesic_dists"] = recognition.calc_geodesic_distances_downsampled_old(demo["cloud_xyz"], demo["cloud_xyz_ds"], ds_inds)
            
        if args.count_steps:
            self.count2segnames = defaultdict(list)
            for (name, demo) in demos.items():
                self.count2segnames[int(demo["seg_index"])].append(name)
            
        rospy.loginfo("done")

    def execute(self,userdata):
        """
        - lookup closest trajectory from database
        - if it's a terminal state, we're done
        - warp it based on the current rope
        returns: done, not_done, failure
        """
        xyz_new = np.squeeze(np.asarray(userdata.points))
        #if args.obj == "cloth": xyz_new = voxel_downsample(xyz_new, .025)
        
        xyz_new_ds, ds_inds = downsample(xyz_new)
        dists_new = recognition.calc_geodesic_distances_downsampled_old(xyz_new,xyz_new_ds, ds_inds)
        
        if args.human_select_demo:
            raise NotImplementedError
            seg_name = trajectory_library.interactive_select_demo(demos)
            best_demo = demos[seg_name]         
            pts0,_ = best_demo["cloud_xyz_ds"]
            pts1,_ = downsample(xyz_new)
            self.f = registration.tps_rpm(pts0, pts1, plotting = 4, reg_init=1,reg_final=args.reg_final,n_iter=40)                            
        else:            
            
            if args.count_steps: candidate_demo_names = self.count2segnames[Globals.stage]
            else: candidate_demo_names = demos.keys()
            
            from joblib import parallel
            
            costs_names = parallel.Parallel(n_jobs=-2)(parallel.delayed(calc_seg_cost)(seg_name, xyz_new_ds, dists_new) for seg_name in candidate_demo_names)
            #costs_names = [calc_seg_cost(seg_name, xyz_new_ds, dists_new) for seg_name in candidate_demo_names]
            #costs_names = [calc_seg_cost(seg_name) for seg_name in candidate_demo_names]
            _, best_name = min(costs_names)

        best_demo = demos[best_name]
        if best_demo["done"]: 
            rospy.loginfo("best demo was a 'done' state")
            return "done"
            
        best_demo = demos[best_name]
        rospy.loginfo("best segment name: %s", best_name)
        xyz_demo_ds = best_demo["cloud_xyz_ds"]
        
        if args.test: n_iter = 21
        else: n_iter = 101
        if args.use_rigid:
            self.f = registration.Translation2d()
            self.f.fit(xyz_demo_ds, xyz_new_ds)
        else:
            self.f = registration.tps_rpm(xyz_demo_ds, xyz_new_ds, plotting = 20, reg_init=1,reg_final=.01,n_iter=n_iter,verbose=False)#, interactive=True)
            np.savez('registration_data', xyz_demo_ds=xyz_demo_ds, xyz_new_ds=xyz_new_ds)
            # print 'correspondences', self.f.corr_nm



        #################### Generate new trajectory ##################
        
        #### Plot original and warped point clouds #######
        # orig_pose_array = conversions.array_to_pose_array(np.squeeze(best_demo["cloud_xyz_ds"]), "base_footprint")
        # warped_pose_array = conversions.array_to_pose_array(self.f.transform_points(np.squeeze(best_demo["cloud_xyz_ds"])), "base_footprint")
        # Globals.handles.append(Globals.rviz.draw_curve(orig_pose_array,rgba=(1,0,0,1),id=19024,type=Marker.CUBE_LIST))
        # Globals.handles.append(Globals.rviz.draw_curve(warped_pose_array,rgba=(0,1,0,1),id=2983,type=Marker.CUBE_LIST))

        #### Plot grid ########
        mins = np.squeeze(best_demo["cloud_xyz"]).min(axis=0)
        maxes = np.squeeze(best_demo["cloud_xyz"]).max(axis=0)
        mins[2] -= .1
        maxes[2] += .1
        grid_handle = warping.draw_grid(Globals.rviz, self.f.transform_points, mins, maxes, 'base_footprint')
        Globals.handles.append(grid_handle)
        
        #### Actually generate the trajectory ###########
        warped_demo = warping.transform_demo_with_fingertips(self.f, best_demo)
        if yes_or_no('dump warped demo?'):
            import pickle
            fname = '/tmp/warped_demo_' + str(np.random.randint(9999999999)) + '.pkl'
            with open(fname, 'w') as f:
                pickle.dump(warped_demo, f)
            print 'saved to', fname

        Globals.pr2.update_rave() 
        trajectory = {}

        # calculate joint trajectory using IK
        for lr in "lr":
            leftright = {"l":"left","r":"right"}[lr]
            if best_demo["arms_used"] in [lr, "b"]:
                if args.hard_table:
                    clipinplace(warped_demo["l_gripper_tool_frame"]["position"][:,2],Globals.table_height+.032,np.inf)
                    clipinplace(warped_demo["r_gripper_tool_frame"]["position"][:,2],Globals.table_height+.032,np.inf)
                arm_traj, feas_inds = lfd_traj.make_joint_traj_by_graph_search(
                    warped_demo["%s_gripper_tool_frame"%lr]["position"],
                    warped_demo["%s_gripper_tool_frame"%lr]["orientation"],
                    Globals.pr2.robot.GetManipulator("%sarm"%leftright),
                    "%s_gripper_tool_frame"%lr,
                    check_collisions=True
                )
                if len(feas_inds) == 0: return "failure"
                trajectory["%s_arm"%lr] = arm_traj
                trajectory["%s_grab"%lr] = best_demo["%s_gripper_joint"%lr] < .07
                trajectory["%s_gripper"%lr] = warped_demo["%s_gripper_joint"%lr]
                trajectory["%s_gripper"%lr][trajectory["%s_grab"%lr]] = 0
        # smooth any discontinuities in the arm traj
        for lr in "lr":
            leftright = {"l":"left","r":"right"}[lr]
            if best_demo["arms_used"] in [lr, "b"]:
                trajectory["%s_arm"%lr], discont_times, n_steps = lfd_traj.smooth_disconts(
                    trajectory["%s_arm"%lr],
                    Globals.pr2.env,
                    Globals.pr2.robot.GetManipulator("%sarm"%leftright),
                    "%s_gripper_tool_frame"%lr
                )
                # after smoothing the arm traj, we need to fill in all other trajectories (in both arms)
                other_lr = 'r' if lr == 'l' else 'l'
                if best_demo["arms_used"] in [other_lr, "b"]:
                    trajectory["%s_arm"%other_lr] = lfd_traj.fill_stationary(trajectory["%s_arm"%other_lr], discont_times, n_steps)
                for tmp_lr in 'lr':
                    if best_demo["arms_used"] in [tmp_lr, "b"]:
                        trajectory["%s_grab"%tmp_lr] = lfd_traj.fill_stationary(trajectory["%s_grab"%tmp_lr], discont_times, n_steps)
                        trajectory["%s_gripper"%tmp_lr] = lfd_traj.fill_stationary(trajectory["%s_gripper"%tmp_lr], discont_times, n_steps)
                        trajectory["%s_gripper"%tmp_lr][trajectory["%s_grab"%tmp_lr]] = 0
        # plotting
        for lr in "lr":
            leftright = {"l":"left","r":"right"}[lr]
            if best_demo["arms_used"] in [lr, "b"]:
                # plot warped trajectory
                Globals.handles.append(Globals.rviz.draw_curve(
                  conversions.array_to_pose_array(
                    alternate(warped_demo["%s_gripper_l_finger_tip_link"%lr]["position"], warped_demo["%s_gripper_r_finger_tip_link"%lr]["position"]),
                    "base_footprint"
                  ),
                  width=.001, rgba = (1,0,1,.4), type=Marker.LINE_LIST,
                  ns='warped_finger_traj'
                ))
                # plot original trajectory
                Globals.handles.append(Globals.rviz.draw_curve(
                  conversions.array_to_pose_array(
                    alternate(best_demo["%s_gripper_l_finger_tip_link"%lr]["position"], best_demo["%s_gripper_r_finger_tip_link"%lr]["position"]),
                    "base_footprint"
                  ),
                  width=.001, rgba = (0,1,1,.4), type=Marker.LINE_LIST,
                  ns='demo_finger_traj'
                ))

        userdata.trajectory = trajectory

        if args.prompt_before_motion:
            consent = yes_or_no("trajectory ok?")
        else:
            consent = True
        
        if consent: return "not_done"
        else: return "failure"
            
def clipinplace(x,lo,hi):
    np.clip(x,lo,hi,out=x)
                
class ExecuteTrajectory(smach.State):
    def __init__(self):
        """
        - first show trajectory in rviz and see if it's ok
        - then execute it
        returns: success, failure
        """
        smach.State.__init__(self, 
            outcomes = ["success", "failure"],
            input_keys = ["trajectory"],
            output_keys = [])
            

    def execute(self, userdata):
        raw_input('about to execute')
        #if not args.test: draw_table()        
        Globals.pr2.update_rave()
        if yes_or_no('about to execute trajectory. save?'):
            import pickle
            fname = '/tmp/trajectory_' + str(np.random.randint(9999999999)) + '.pkl'
            with open(fname, 'w') as f:
                pickle.dump(userdata.trajectory, f)
            print 'saved to', fname
        success = lfd_traj.follow_trajectory_with_grabs(Globals.pr2, userdata.trajectory)
        raw_input('done executing segment. press enter to continue')
        if success: 
            if args.count_steps: Globals.stage += 1
            return "success"
        else: return "failure"
        
def make_tie_knot_sm():
    sm = smach.StateMachine(outcomes = ["success", "failure"])
    with sm:
        smach.StateMachine.add("look_at_object", LookAtObject(), transitions = {"success":"select_traj", "failure":"failure"})
        smach.StateMachine.add("select_traj", SelectTrajectory(), transitions = {"done":"success","not_done":"execute_traj", "failure":"failure"})
        smach.StateMachine.add("execute_traj", ExecuteTrajectory(), transitions = {"success":"look_at_object","failure":"look_at_object"})
        
        
    return sm

if __name__ == "__main__":
    Globals.handles = []
    if rospy.get_name() == '/unnamed':
        rospy.init_node("tie_knot",disable_signals=True)
    Globals.setup()
    #Globals.pr2.torso.go_up()
    #Globals.pr2.head.set_pan_tilt(0, HEAD_TILT)
    if args.use_tracking:
        Globals.pr2.larm.goto_posture('side')
        Globals.pr2.rarm.goto_posture('side')        
    Globals.pr2.join_all()
    tie_knot_sm = make_tie_knot_sm()
    tie_knot_sm.execute()
