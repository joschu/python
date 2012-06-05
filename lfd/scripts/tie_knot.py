import roslib; roslib.load_manifest("smach_ros")
import smach
from lfd import registration, trajectory_library, warping
import lfd
from kinematics import reachability
from utils.yes_or_no import yes_or_no
import sensor_msgs.msg
import geometry_msgs.msg as gm
import rospy
from image_proc import curves, interactive_roi as roi
from brett2 import ros_utils, PR2, trajectories as traj
import numpy as np
from utils import conversions
from knot_tying import tps


RVIZ_LPOS = 0
RVIZ_RPOS = 1

HUMAN_SELECT_DEMO=True
HUMAN_GET_ROPE=True

def human_get_rope():
    point_cloud = rospy.wait_for_message("/camera/depth_registered/points", sensor_msgs.msg.PointCloud2)
    
    xyz, bgr = ros_utils.pc2xyzrgb(point_cloud)
    
    xys = roi.get_polyline(bgr,'draw curve')
    uvs = np.int32(xys)[:,::-1]
    us,vs = uvs.T
    xyzs = xyz[us,vs]
    xyzs_good = xyzs[np.isfinite(xyzs).all(axis=1)]
    print "%i out of %i labeled points are good"%(len(xyzs_good), len(xyzs))
    xyzs_unif = curves.unif_resample(xyzs_good,100,tol=.002)
    return xyzs_unif, point_cloud.header.frame_id
    
    
def fix_gripper(g):
    g = g/7
    g[g < .03] = 0
    return g

def fit_tps(pts0, pts1):
    pts0 = np.r_[pts0, np.zeros((1,3))]
    pts1 = np.r_[pts1, np.zeros((1,3))]    
    if np.sign(pts0[-1,1] - pts0[0,1]) != np.sign(pts1[-1,1] - pts1[0,1]):
        pts1 = pts1[::-1]    
    f = tps.TPS33(pts0[:,0], pts0[:,1], pts0[:,2], pts1[:,0], pts1[:,1], pts1[:,2])
    return f

class Globals:
    pr2 = None
    rviz = None
    
    def __init__(self): raise

    @staticmethod
    def setup():
        Globals.pr2 = PR2.PR2()
        Globals.rviz = ros_utils.RvizWrapper()

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
        
        Globals.pr2.head.set_pan_tilt(0, .7)
        if HUMAN_GET_ROPE:
            xyz,frame = human_get_rope()
            xyz = ros_utils.transform_points(xyz, Globals.pr2.tf_listener, "base_footprint", frame)
        else:
            msg = rospy.wait_for_message("/preprocessor/points", sensor_msgs.msg.PointCloud2)
            xyz, rgb = ros_utils.pc2xyzrgb(msg)
            xyz = xyz.reshape(-1,3)
            xyz = ros_utils.transform_points(xyz, Globals.pr2.tf_listener, "base_footprint", msg.header.frame_id)
        userdata.points = xyz
        return "success"
        
class SelectTrajectory(smach.State):
    f = None
    def __init__(self):
        smach.State.__init__(self, 
            outcomes = ["done", "not_done","failure"],
            input_keys = ["points"],
            output_keys = ["trajectory", "left_used", "right_used"])
            
        self.library = trajectory_library.TrajectoryLibrary("lm.h5", "read")
        self.db = reachability.ReachabilityDatabase("read");

    def execute(self,userdata):
        """
        - lookup closest trajectory from database
        - if it's a terminal state, we're done
        - warp it based on the current rope
        returns: done, not_done, failure
        
        visualization: 
        - show all library states
        - warping visualization from matlab
        """
        if HUMAN_SELECT_DEMO:
            #seg_name = trajectory_library.interactive_select_demo(self.library)
            seg_name = "demo0.0"
        else:
            raise NotImplementedError

        if seg_name == "stop": return "done"
        else: demo = self.library.root["segments"][seg_name]

        if self.f is None:
            self.f = registration.NonrigidRegistrationMatlab(display=True)
        #from image_proc.clouds import voxel_downsample
        self.f.fit_transformation(np.r_[demo["rope"][0],np.zeros((1,3))], 
                                  np.r_[userdata.points,np.zeros((1,3))])
                                          #np.r_[voxel_downsample(userdata.points,.03),np.zeros((1,3))])
        
        #self.f = fit_tps(demo["rope"][0], userdata.points)
        
        userdata.left_used = left_used = (demo["grab_l"] > -1).any()
        userdata.right_used = right_used = (demo["grab_r"] > -1).any()
        
        warped_demo = warping.transform_demo(self.f, demo, left=left_used, right=right_used)
        trajectory = np.zeros(len(demo), dtype=traj.BodyState)                        
                
        if left_used:
            trajectory["l_arm"] = traj.make_joint_traj(warped_demo["xyz_l"], warped_demo["quat_l"], Globals.pr2.robot.GetManipulator("leftarm"),18)            
            trajectory["l_gripper"] = fix_gripper(warped_demo["grip_l"])
            Globals.rviz.draw_curve(conversions.array_to_pose_array(warped_demo["xyz_l"], "base_footprint"), RVIZ_LPOS, width=.01, rgba = (1,0,0,1))
        if right_used:
            trajectory["r_arm"] = traj.make_joint_traj(warped_demo["xyz_r"], warped_demo["quat_r"], Globals.pr2.robot.GetManipulator("rightarm"),18)            
            trajectory["r_gripper"] = fix_gripper(warped_demo["grip_r"])
            Globals.rviz.draw_curve(conversions.array_to_pose_array(warped_demo["xyz_r"], "base_footprint"), RVIZ_RPOS, width=.01, rgba = (1,0,0,1))
        userdata.trajectory = trajectory
        #userdata.base_xya = (x,y,0)
        # todo: draw pr2        
        consent = yes_or_no("trajectory ok?")
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
        traj.follow_body_traj(Globals.pr2, userdata.trajectory, np.linspace(0,5, len(userdata.trajectory)),
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
        smach.StateMachine.add("execute_traj", ExecuteTrajectory(), transitions = {"success":"success","failure":"failure"})
        
        
    return sm

if __name__ == "__main__":
    if rospy.get_name() == '/unnamed':
        rospy.init_node("tie_knot")
    Globals.setup()
    rospy.sleep(1)
    tie_knot_sm = make_tie_knot_sm()
    tie_knot_sm.execute()