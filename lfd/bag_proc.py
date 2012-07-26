import rosbag
from brett2 import mytf
from copy import deepcopy
import numpy as np
import openravepy
from utils.func_utils import once
from brett2 import ros_utils
from utils import conversions

MIN_TIME = .1
MIN_JOINT_CHANGE = .0025

class RosToRave(object):
    
    @once
    def create():
        return RosToRave()
    
    def __init__(self):
        self.env = openravepy.Environment()
        self.env.Load("robots/pr2-beta-static.zae")
        self.robot = self.env.GetRobot("pr2")
        self.initialized = False
        
    def init(self, joint_msg):

        self.ros_names = joint_msg.name
        inds_ros2rave = np.array([self.robot.GetJointIndex(name) for name in self.ros_names])
        self.good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
        self.rave_inds = inds_ros2rave[self.good_ros_inds] # openrave indices corresponding to those joints

        ros_values = joint_msg.position        
        rave_values = [ros_values[i_ros] for i_ros in self.good_ros_inds]        
        self.robot.SetJointValues(rave_values[:20],self.rave_inds[:20])
        self.robot.SetJointValues(rave_values[20:],self.rave_inds[20:])          
        
        self.initialized = True

    def update(self, joint_msg):
        ros_values = joint_msg.position        
        rave_values = [ros_values[i_ros] for i_ros in self.good_ros_inds]        
        self.robot.SetJointValues(rave_values[:20],self.rave_inds[:20])
        self.robot.SetJointValues(rave_values[20:],self.rave_inds[20:])          


def extract_kinematics_from_bag(bag, link_names, joint_names):    
    """
    Input bagfile has the following topics: /tf, /joint_states, /joy, /xxx/points
    Output array has the following fields: /joints, [frames_list]
    """    
    
    rr = RosToRave.create()
    robot = rr.robot
        
    links = [robot.GetLink(name) for name in link_names]
    joints = [robot.GetJoint(name) for name in joint_names]

    
    out = {}
    for link_name in link_names:
        out[link_name] = {}
        out[link_name]["position"] = []
        out[link_name]["orientation"] = []
        out[link_name]["hmat"] = []
    for joint_name in joint_names:
        out[joint_name] = []
    out["joints"] = []
    out["time"] = []
    
    first_message = True
    for (topic, msg, t) in bag.read_messages(topics=['/joint_states']):

        t = t.to_sec()
        if first_message:
            rr.init(msg)
            out["joint_names"] = msg.name
            prev_positions = np.ones(len(msg.position))*100
            prev_t = -np.inf
            first_message = False        
                    
        if t - prev_t > MIN_TIME and np.any(np.abs(np.array(msg.position) - prev_positions) > MIN_JOINT_CHANGE):
            
            rr.update(msg)
            
            out["joints"].append(msg.position)
            out["time"].append(t)
            
            for (link_name, link) in zip(link_names,links):
                hmat = link.GetTransform()
                xyz, quat = conversions.hmat_to_trans_rot(hmat)
                out[link_name]["position"].append(xyz)
                out[link_name]["orientation"].append(quat)
                out[link_name]["hmat"].append(hmat)
                            
            prev_t = t
            prev_positions = np.array(msg.position)
            

    convert_lists_to_arrays(out)
    return out

def convert_lists_to_arrays(D):
    for (d,key,val) in walk_through_recursive_dict(D):
        if isinstance(val, dict):
            convert_lists_to_arrays(val)
        elif isinstance(val, list):
            d[key] = np.array(val)
                
def walk_through_recursive_dict(D):
    for (key, val) in D.items():
        if isinstance(val, dict):
            for dkv in walk_through_recursive_dict(val):
                yield dkv
        else:
            yield (D,key,val)
    
def get_button_presses(bag):    
    last_msg = None    
    presses = []
    
    for (topic, msg, t) in bag.read_messages(topics=['/joy']):
        if last_msg is None:
            continue
        else:
            for i in xrange(16):
                if msg.buttons[i] and not last_msg.buttons[i]:
                    presses.append((t,i))
            last_msg = msg
        last_msg = msg    
        
    return presses

def get_transformed_clouds(bag,times):
    """
    get transformed point clouds preceeding times    
    """
    cloud_times = []
    for (topic, msg, t) in bag.read_messages():
        if hasattr(msg, "row_step"):
            cloud_times.append(t)
            
    cloud_times = np.array(cloud_times)
    cloud_inds = set(np.searchsorted(cloud_times, times))
    
    listener = mytf.TransformListener()

    cloud_xyzs = []
    cloud_bgrs = []
    
    for (i,(topic, msg, t)) in enumerate(bag.read_messages()):
        if topic=="/tf":
            listener.callback(msg)
        if i in cloud_inds:
            xyz, bgr = ros_utils.pc2xyzrgb(msg)
            
            xyz_tf = ros_utils.transform_points(xyz, listener, "base_footprint", msg.header.frame_id)
            cloud_xyzs.append(xyz_tf)
            cloud_bgrs.append(bgr)  
            
    return cloud_xyzs, cloud_bgrs    
    
    
def create_segments(bag, link_names, joint_names):    
    
    listener = mytf.TransformListener()
    
    t_start_bag = bag.read_messages().next()[2].to_sec()

    button_presses = get_button_presses(bag)
    
    start_times, stop_times, look_times, l_close_times, l_open_times, r_close_times, r_open_times = [],[],[],[],[],[],[]
    for (time, button) in button_presses:
        if button == 12: look_times.append(time)
        elif button == 4: start_times.append(time)
        elif button == 6: stop_times.append(time)
        elif button == 7: l_open_times.append(time)
        elif button == 5: l_close_times.append(time)
        elif button == 15: r_open_times.append(time)
        elif button == 13: r_close_times.append(time)
        
    
    kinematics_info = extract_kinematics_from_bag(bag, link_names, joint_names)
    
    assert(len(look_times)==len(start_times)==len(stop_times))
        
    N = len(kinematics_info["time"])
    
    l_open = np.zeros(N, bool)
    l_close = np.zeros(N, bool)
    r_open = np.zeros(N, bool)
    r_close = np.zeros(N, bool)
    
    times = kinematics_info["time"]
    l_open[np.searchsorted(times, l_open_times)] = True
    l_close[np.searchsorted(times, l_close_times)] = True
    r_open[np.searchsorted(times, r_open_times)] = True
    r_close[np.searchsorted(times, r_close_times)] = True
            
    kinematics_info["l_open"] = l_open
    kinematics_info["l_close"] = l_close
    kinematics_info["r_open"] = r_open
    kinematics_info["r_close"] = r_close
    
    start_inds = np.searchsorted(times, start_times)
    stop_inds = np.searchsorted(times, stop_times)
    
    seg_infos = []
    
    for (start_ind, stop_ind) in zip(start_inds, stop_inds):
        seg_info = extract_segment(kinematics_info, start_ind, stop_ind)
        seg_infos.append(seg_info)
        
    return seg_infos
        
        
def extract_segment(kin_info, start, stop):
    seg_info = deepcopy(kin_info)
    for (d,key,val) in walk_through_recursive_dict(kin_info):
        d[key] = val[start:stop]
    return seg_info

