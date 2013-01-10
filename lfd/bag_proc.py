"""
Functions that enable you to extract kinematics info from a bag file
and put it in an array with a bunch of time serieses
"""

from __future__ import division
from brett2 import mytf
import numpy as np
import openravepy
from jds_utils.func_utils import once
from brett2 import ros_utils
from jds_utils import conversions
from jds_utils.math_utils import norms
import sensor_msgs.msg as sm
from pprint import pprint

MIN_TIME = .025
MIN_JOINT_CHANGE = .001
    
    
class RosToRave(object):
    
    "take in ROS joint_states messages and use it to update an openrave robot"
    
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


def extract_kinematics_from_bag(bag, link_names):    
    """
    Input bagfile has the following topics: /tf, /joint_states, /joy, /xxx/points
    Output dictionary has a bunch of arrays with kinematics info, e.g. 
    /r_gripper_tool_frame/position
                          orientation
                          hmat
    /joint_states/position
                  orientation
    etc.
    """    
    
    rr = RosToRave.create()
    robot = rr.robot
        
    links = [robot.GetLink(name) for name in link_names]
    
    l_joint_inds = robot.GetManipulator("leftarm").GetArmIndices()
    r_joint_inds = robot.GetManipulator("rightarm").GetArmIndices()
    l_gripper_inds = [robot.GetJointIndex("l_gripper_joint")]
    r_gripper_inds = [robot.GetJointIndex("r_gripper_joint")]
    
    out = {}
    for link_name in link_names:
        out[link_name] = {}
        out[link_name]["position"] = []
        out[link_name]["orientation"] = []
        out[link_name]["hmat"] = []
    out["joint_states"] = {}
    out["joint_states"]["position"] = []
    out["leftarm"] = []
    out["rightarm"] = []
    out["l_gripper_joint"] = []
    out["r_gripper_joint"] = []

    out["time"] = []
    
    first_message = True
    for (topic, msg, t) in bag.read_messages(topics=['/joint_states']):

        t = t.to_sec()
        if first_message:
            rr.init(msg)
            out["joint_states"]["name"] = msg.name
            prev_positions = np.ones(len(msg.position))*100
            prev_t = -np.inf
            first_message = False        
                    
        if t - prev_t > MIN_TIME and np.any(np.abs(np.array(msg.position) - prev_positions) > MIN_JOINT_CHANGE):
            
            rr.update(msg)
            
            out["joint_states"]["position"].append(msg.position)
            out["time"].append(t)
            
            for (link_name, link) in zip(link_names,links):
                hmat = link.GetTransform()
                xyz, quat = conversions.hmat_to_trans_rot(hmat)
                out[link_name]["position"].append(xyz)
                out[link_name]["orientation"].append(quat)
                out[link_name]["hmat"].append(hmat)
                
            robot.SetActiveDOFs(l_joint_inds)
            out["leftarm"].append(robot.GetActiveDOFValues())
            robot.SetActiveDOFs(r_joint_inds)
            out["rightarm"].append(robot.GetActiveDOFValues())


            robot.SetActiveDOFs(l_gripper_inds)
            out["l_gripper_joint"].append(robot.GetActiveDOFValues()[0])
            robot.SetActiveDOFs(r_gripper_inds)
            out["r_gripper_joint"].append(robot.GetActiveDOFValues()[0])
                            
            prev_t = t
            prev_positions = np.array(msg.position)
            

    out = convert_lists_to_arrays(out)
    return out

def convert_lists_to_arrays(D):
    return map_dict(np.array, D)


def map_dict(fn, D):
    "apply fn to every 'leaf' element of dictionary"
    out = {}
    for (path, val) in dfs_dict(D):
        d = out
        for key in path[:-1]:
            d = d[key]
        if isinstance(val, dict):
            d[path[-1]] = {}
        else:
            d[path[-1]] = fn(val)
    return out
    

def dfs_dict(D):
    "iterator returning leaves of dictionary, resulting from depth-first search"
    for (key, val) in D.items():
        if isinstance(val, dict):
            yield (key,), val
            for (path, obj) in dfs_dict(val):
                yield (key,)+path, obj
        else:
            yield (key,), val

def get_button_presses(bag):    
    "extract button presses from bag file"
    last_msg = None    
    presses = []
    
    for (topic, msg, t) in bag.read_messages(topics=['/joy']):
        if last_msg is not None:
            for i in xrange(16):
                if msg.buttons[i] and not last_msg.buttons[i]:
                    presses.append((t.to_sec(),i))
            last_msg = msg
        last_msg = msg    
        
    return presses

def get_transformed_clouds(bag,times):
    """
    get transformed point clouds at times    
    """
    listener = mytf.TransformListener()

    all_clouds = []
    all_cloud_times = []
    all_hmats = []
    for (i,(topic, msg, t)) in enumerate(bag.read_messages()):
        if topic=="/tf":
            listener.callback(msg)
        elif topic.endswith("points"):
            xyz, bgr = ros_utils.pc2xyzrgb(msg)            
            all_clouds.append((xyz, bgr))
            try:
                all_hmats.append(listener.transformer.lookup_transform_mat("/base_footprint",msg.header.frame_id))
            except Exception:
                all_hmats.append(np.eye(4))
            all_cloud_times.append(t.to_sec())
    
    cloud_inds = np.searchsorted(all_cloud_times, times)
    
    clouds = []
    for i in cloud_inds:
        xyz,rgb = all_clouds[i]
        hmat = all_hmats[i]
        xyz1 = np.c_[xyz.reshape(-1,3), np.ones((xyz.size/3,1))]
        xyz1_tf = np.dot(xyz1, hmat.T)
        xyz_tf = xyz1_tf[:,:3].reshape(xyz.shape)
        clouds.append((xyz_tf, rgb))

    return clouds
    
    
def create_segments(bag, link_names):    
    """extract a bunch of dictionaries from bag file, each one corresponding to one segment of trajectory. 
    each segment corresponds to 'look', 'start', and 'stop' button presses"""
    button_presses = get_button_presses(bag)

    def actions_to_str(all_times):
        CHARS = {
            'look': '*',
            'start': '[',
            'stop': ']',
            'l_open': 'L',
            'l_close': 'l',
            'r_open': 'R',
            'r_close': 'r',
            'done': '.',
        }
        return ''.join(CHARS[a] for _, a in all_times)

    all_times = []
    for (time, button) in button_presses:
        if button == 12: all_times.append((time, 'look'))
        elif button == 0: all_times.append((time, 'start'))
        elif button == 3: all_times.append((time, 'stop'))
        elif button == 7: all_times.append((time, 'l_open'))
        elif button == 5: all_times.append((time, 'l_close'))
        elif button == 15: all_times.append((time, 'r_open'))
        elif button == 13: all_times.append((time, 'r_close'))
        elif button == 14: all_times.append((time, 'done'))
    all_times.sort()
    print 'Events in bag file:'; pprint(all_times)
    print actions_to_str(all_times)

    # Try to correct errors in start/stop/look
    corrected_all_times = []

    # First find the segment start/stops. They should come in pairs.
    start_times = [p[0] for p in all_times if p[1] == 'start']
    stop_times = [p[0] for p in all_times if p[1] == 'stop']
    assert len(start_times) == len(stop_times)
    segments = zip(start_times, stop_times) # TODO: check that the segments make sense + error recovery
    def get_containing_seg(segments, t):
        for i, (sstart, send) in enumerate(segments):
            if sstart <= t <= send:
                return i
        return -1

    # remove gripper open/closes that don't happen in a segment
    for i in range(len(all_times)):
        if all_times[i][1] in ['l_open', 'l_close', 'r_open', 'r_close'] and get_containing_seg(segments, all_times[i][0]) == -1:
            print "WARNING: ignoring button press '%s' at time %s (because this should occur within a segment)" % \
                (all_times[i][1], all_times[i][0])
            continue
        corrected_all_times.append(all_times[i])
    all_times = corrected_all_times; corrected_all_times = []
    print 'After ignoring extra open/close events:'; pprint(all_times)
    print actions_to_str(all_times)

    # first, as a heuristic, only choose 'done's that occur at the very end (there should be only one)
    for i in range(len(all_times)):
        if i < len(all_times) - 1 and all_times[i][1] == 'done':
            print "WARNING: ignoring button press '%s' at time %s (because 'done' should only happen at the end)" % \
                (all_times[i][1], all_times[i][0])
            continue
        corrected_all_times.append(all_times[i])
    # we might've removed all the done events, so insert a fake one at the end if necessary
    if corrected_all_times[-1][1] != 'done':
        new_time = corrected_all_times[-1][0] + 1.0
        print "WARNING: inserting artificial 'done' at time %s (because the last event should be a 'done')" % \
                (new_time)
        corrected_all_times.append((new_time, 'done'))
    all_times = corrected_all_times; corrected_all_times = []
    print 'After ignoring extra done events:'; pprint(all_times)
    print actions_to_str(all_times)

    # only choose the 'look's that occur right before the 'start's
    # (this way the view probably won't be occluded...)
    for i in range(len(all_times)):
        if i < len(all_times) - 1 and all_times[i][1] == 'look' and all_times[i+1][1] != 'start':
            print "WARNING: ignoring button press '%s' at time %s (because the next action is not 'start')" % \
                (all_times[i][1], all_times[i][0])
            continue
        corrected_all_times.append(all_times[i])
    all_times = corrected_all_times; corrected_all_times = []
    print 'After ignoring extra look events:'; pprint(all_times)
    print actions_to_str(all_times)

    start_times = [t for t, action in all_times if action == 'start']
    stop_times = [t for t, action in all_times if action == 'stop']
    look_times = [t for t, action in all_times if action == 'look']
    l_close_times = [t for t, action in all_times if action == 'l_close']
    l_open_times = [t for t, action in all_times if action == 'l_open']
    r_close_times = [t for t, action in all_times if action == 'r_close']
    r_open_times = [t for t, action in all_times if action == 'r_open']
    done_times = [t for t, action in all_times if action == 'done']

    assert len(start_times) == len(stop_times) == len(look_times)
    assert len(done_times) == 1


    kinematics_info = extract_kinematics_from_bag(bag, link_names)

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


    look_clouds = get_transformed_clouds(bag, look_times)
    
    seg_infos = []
    
    for (start_ind, stop_ind, (xyz, bgr)) in zip(start_inds, stop_inds, look_clouds):
        seg_info = extract_segment(kinematics_info, start_ind, stop_ind)
        seg_info["arms_used"] = determine_arms_used(seg_info)
        seg_info["cloud_xyz"] = xyz
        seg_info["cloud_bgr"] = bgr
        seg_info["done"] = False
        seg_infos.append(seg_info)
        
    done_clouds = get_transformed_clouds(bag, done_times)
    for (xyz, bgr) in done_clouds:
        seg_info = {}
        seg_info["cloud_xyz"] = xyz
        seg_info["cloud_bgr"] = bgr
        seg_info["done"] = True
        seg_infos.append(seg_info)
        
    return seg_infos
        

def create_segment_without_look(bag, link_names):    
    """
    basically same as 'create_segments' above except no look times
    """
    button_presses = get_button_presses(bag)
    
    start_times, stop_times, look_times, l_close_times, l_open_times, r_close_times, r_open_times, done_times = [],[],[],[],[],[],[],[]
    for (time, button) in button_presses:
        if button == 0: start_times.append(time)
        elif button == 3: stop_times.append(time)
        
    kinematics_info = extract_kinematics_from_bag(bag, link_names)
    
    assert len(start_times)==len(stop_times)

    times = kinematics_info["time"]
    start_inds = np.searchsorted(times, start_times)
    stop_inds = np.searchsorted(times, stop_times)

    
    seg_infos = []
        
    for (start_ind, stop_ind) in zip(start_inds, stop_inds):
        seg_info = extract_segment(kinematics_info, start_ind, stop_ind)
        seg_infos.append(seg_info)
        
        
    return seg_infos                
        
        
def path_length(x):
    """
    sum of absolute values of all differences between time steps
    """
    if x.ndim == 1: x = x.reshape(-1,1)
    return norms(x[1:] - x[:-1],1).sum()        
        
def determine_arms_used(segment):
    r_xyz = segment["r_gripper_tool_frame"]["position"]        
    l_xyz = segment["l_gripper_tool_frame"]["position"]
    r_gripper = segment["r_gripper_joint"]
    l_gripper = segment["l_gripper_joint"]
    
    right_used = path_length(r_xyz) > .05 or path_length(r_gripper) > .04
    left_used = path_length(l_xyz) > .05 or path_length(l_gripper) > .04
    
    if right_used and not left_used: return "r"
    elif left_used and not right_used: return "l"
    elif right_used and left_used: return "b"
    else:
        raise Exception("I can't figure out which arm was used")

def extract_segment(kin_info, start, stop):
    """
    extract portion between two indices
    kind of bad because some arrays in kin_info are not time serieses
    and it's not clear if this function will always do the right thing with them.
    """
    # XXX: might do the wrong thin on non-timeseries members
    def fn(x):
        if x.dtype.type == np.string_:
            return x
        else:
            return x[start:stop]
    return map_dict(fn, kin_info)

def dict_to_hdf(group, data, name):
    """
    will create a child of 'group' called 'name' with all of the fields in 'data' dict
    """
    if isinstance(data, dict):
        group.create_group(name)
        for (key, val) in data.items():
            dict_to_hdf(group[name], val, key)
    else:
        group[name] = data
