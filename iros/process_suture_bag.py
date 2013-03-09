#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("task")
parser.add_argument("part_name")
args = parser.parse_args()


import lfd
import iros
from lfd import bag_proc as bp
import rosbag
import rospy
import os.path as osp
from jds_utils.yes_or_no import yes_or_no
import brett2.ros_utils as ru
from jds_utils.colorize import colorize
import os, sys
import yaml
import h5py
import numpy as np
import cv2
import simple_clicker as sc
import feature_utils as fu

# find data files, files to save to
SAVE_TRAJ = False
IROS_DATA_DIR = os.getenv("IROS_DATA_DIR")

task_file = osp.join(IROS_DATA_DIR, "suture_demos.yaml")

with open(osp.join(IROS_DATA_DIR,task_file),"r") as fh:
    task_info = yaml.load(fh)

bag = task_info[args.task][args.part_name]["demo_bag"]
bag = rosbag.Bag(bag)

jtf = osp.join(IROS_DATA_DIR, 'joint_trajectories', args.task, 'pt' + str(task_info[args.task][args.part_name]["pt_num"]))
kpf = osp.join(IROS_DATA_DIR, 'key_points', args.task, 'pt' + str(task_info[args.task][args.part_name]["pt_num"]))
pcf = osp.join(IROS_DATA_DIR, 'point_clouds', args.task, 'pt' + str(task_info[args.task][args.part_name]["pt_num"]))

### extract kinematics info from bag
def extract_kinematics(np_file, info):
    
    for i in range(len(info)):
        savefile = np_file + 'seg%s'%i
        print 'Writing to', savefile

        if osp.exists(savefile + '_larm.npy'):
            if yes_or_no(savefile + ' already exists. Overwrite?'):
                os.remove(savefile + '_larm.npy')
                os.remove(savefile + '_rarm.npy')
                os.remove(savefile + '_lgrip.npy')
                os.remove(savefile + '_rgrip.npy')
            else:
                print 'Aborting...'
                sys.exit(1)

        larm = info[i]["leftarm"]
        rarm = info[i]["rightarm"]
        lgrip = info[i]["l_gripper_joint"]
        rgrip = info[i]["r_gripper_joint"]

        np.save(savefile + '_larm.npy' , np.array(larm))
        np.save(savefile + '_rarm.npy' , np.array(rarm))
        np.save(savefile + '_lgrip.npy' , np.array(lgrip))
        np.save(savefile + '_rgrip.npy' , np.array(rgrip))

    print 'saved all'
    
       
# creates the text for the yaml file for the demo
def get_new_demo_entry_text(segment, hole1, hole2, tcut, mcut, bcut, needle):

    new_entry_text = """        
        segment%(seg_num)s:
            hole1 center:  %(hole1)s
            hole2 center:  %(hole2)s
            top of cut:    %(tcut)s
            middle of cut: %(mcut)s
            bottom of cut: %(bcut)s
            needle tip:    %(needle)s
""" % dict(
            seg_num = segment,
            hole1 = hole1,
            hole2 = hole2,
            tcut = tcut,
            mcut = mcut,
            bcut = bcut,
            needle = needle,
            )

    return new_entry_text


# appends the entry text to the yaml file
def add_new_entry_to_yaml_file(data_dir, new_entry_text):
    demofile = open(data_dir + "/suture_demos.yaml", "a")
    demofile.write(new_entry_text)
    demofile.flush()
    os.fsync(demofile.fileno())
    demofile.close()

print 'getting bag info...'

button_presses = bp.get_button_presses(bag)
all_times = bp.read_button_presses(button_presses)
start_times = [t for t, action in all_times if action == 'start']
stop_times = [t for t, action in all_times if action == 'stop']
look_times = [t for t, action in all_times if action == 'look']

assert len(start_times) == len(stop_times)
SEGNUM = len(look_times)


if SAVE_TRAJ == True:
    raw_input("press enter to extract kinematics info from bag file")

    link_names = ["r_gripper_tool_frame", "l_gripper_tool_frame"]             
    kinematics_info = bp.extract_kinematics_from_bag(bag, link_names)    
    times = kinematics_info["time"]

    start_inds = np.searchsorted(times, start_times)
    stop_inds = np.searchsorted(times, stop_times)

    forced_segs_info = []
    for start_ind, stop_ind in zip(start_inds, stop_inds):
        seg = bp.extract_segment(kinematics_info, start_ind, stop_ind)
        forced_segs_info.append(seg)
    
    extract_kinematics(jtf, forced_segs_info)         

    raw_input("extracted segments. press enter to continue")

# determine which keypoints matter for each segment based on user input
print 'getting all look_time point clouds...'
look_clouds = bp.get_transformed_clouds(bag, look_times)

window_name = "Find Keypoints"
keypt_list = ['lh','rh','ct', 'ne', 'nt', 'ntt', 'auto']
keypoints_locations = []
keypoints_names = []

for s in range(SEGNUM):
    print 'look time for segment %s'%s
    keypt_names = []
    keypt_locs = []

    while (True):
        sc.show_pointclouds(look_clouds[s], window_name)
        kp = raw_input("which key points are important for this segment? choices are: " + str(keypt_list) + ". (please only enter one key point at a time): ")
        
        if kp not in keypt_list:
            print 'incorrect input. try again!'
            continue  
        
        elif kp == 'auto':
            print colorize("Looking for Holes and Cut automatically...", 'green', bold=True)

            xyz_tf = look_clouds[s][0].copy()
            rgb_plot = look_clouds[s][1].copy()
            xyz_tf[np.isnan(xyz_tf)] = -2
            
            holes = fu.automatic_find_holes(rgb_plot, args.task)
            
        elif kp == 'lh':
            print colorize("Looking for Left Hole...", 'green', bold=True)
            
            xyz_tf = look_clouds[s][0].copy()
            rgb_plot = look_clouds[s][1].copy()
            xyz_tf[np.isnan(xyz_tf)] = -2 
            np.save(pcf + 'seg%s_lh_xyz_tf.npy'%s, xyz_tf)
            np.save(pcf + 'seg%s_lh_rgb_pl.npy'%s, rgb_plot)

            hole_loc = sc.find_hole('left_hole', xyz_tf, rgb_plot, window_name)
            keypt_locs.append(hole_loc) 
            keypt_names.append('left_hole')
  
        elif kp == 'rh':
            print colorize("Looking for Right Hole...", 'green', bold=True)
            
            xyz_tf = look_clouds[s][0].copy()
            rgb_plot = look_clouds[s][1].copy()
            xyz_tf[np.isnan(xyz_tf)] = -2 
            np.save(pcf + 'seg%s_rh_xyz_tf.npy'%s, xyz_tf)
            np.save(pcf + 'seg%s_rh_rgb_pl.npy'%s, rgb_plot)
    
            hole_loc = sc.find_hole('right_hole', xyz_tf, rgb_plot, window_name)
            keypt_locs.append(hole_loc) 
            keypt_names.append('right_hole')
            
        elif kp == 'ct':
            print colorize("Looking for Cut...", 'green', bold=True)
            
            xyz_tf = look_clouds[s][0].copy()
            rgb_plot = look_clouds[s][1].copy()
            xyz_tf[np.isnan(xyz_tf)] = -2 
            np.save(pcf + 'seg%s_ct_xyz_tf.npy'%s, xyz_tf)
            np.save(pcf + 'seg%s_ct_rgb_pl.npy'%s, rgb_plot)
    
            tcut_loc, mcut_loc, bcut_loc = sc.find_cut(xyz_tf, rgb_plot, window_name)
            keypt_locs.append(tcut_loc)
            keypt_locs.append(mcut_loc)
            keypt_locs.append(bcut_loc)
            keypt_names.append('cut')       
            
        elif kp == 'ne':
            xyz_tfs = []
            rgb_plots = []
            needle_look_times = []            
            num_clouds = 15
            
            for t in range(num_clouds):
                needle_look_times.append(look_times[s] + t)
                
            print colorize("Looking for Needle End...", 'green', bold=True)          
            print 'getting the needle point clouds...'
            needle_clouds = bp.get_transformed_clouds(bag, needle_look_times)

            for i in range(num_clouds):
                xyz_tfs.append(needle_clouds[i][0].copy())
                rgb_plots.append(needle_clouds[i][1].copy())
                xyz_tfs[i][np.isnan(xyz_tfs[i])] = -2 
            np.save(pcf + 'seg%s_ne_xyz_tfs.npy'%s, xyz_tfs)
            np.save(pcf + 'seg%s_ne_rgb_pls.npy'%s, rgb_plots)                
                
            needle_loc = sc.find_needle_end(xyz_tfs, rgb_plots, window_name)
            
            keypt_locs.append(needle_loc)
            keypt_names.append('needle_end')            
            
            del needle_look_times
            del xyz_tfs
            del rgb_plots
            
        elif kp in ['nt', 'ntt']:
            xyz_tfs = []
            rgb_plots = []
            needle_look_times = []            
            num_clouds = 15
            
            for t in range(num_clouds):
                needle_look_times.append(look_times[s] + t)
                
            print colorize("Looking for Needle Tip...", 'green', bold=True)          
            print 'getting the needle point clouds...'
            needle_clouds = bp.get_transformed_clouds(bag, needle_look_times)

            for i in range(num_clouds):
                xyz_tfs.append(needle_clouds[i][0].copy())
                rgb_plots.append(needle_clouds[i][1].copy())
                xyz_tfs[i][np.isnan(xyz_tfs[i])] = -2 
                
            needle_loc = sc.find_needle_tip(xyz_tfs, rgb_plots, window_name)
            
            if kp == 'nt':
                keypt_locs.append(needle_loc)
                keypt_names.append('needle_tip')      
                np.save(pcf + 'seg%s_nt_xyz_tfs.npy'%s, xyz_tfs)
                np.save(pcf + 'seg%s_nt_rgb_pls.npy'%s, rgb_plots)                             
            else:
                keypt_locs.append((0,0,0))
                keypt_names.append('empty')                
                np.save(kpf + '_needle_world_loc.npy', needle_loc)
                np.save(pcf + 'seg%s_ntt_xyz_tfs.npy'%s, xyz_tfs)
                np.save(pcf + 'seg%s_ntt_rgb_pls.npy'%s, rgb_plots)                             
            
            del needle_look_times
            del xyz_tfs
            del rgb_plots                  
            
        if yes_or_no('Key point %s saved for this segment. Is there another key point for this segment?'%kp):
            continue    
        else:
            keypoints_locations.append(keypt_locs)
            keypoints_names.append(keypt_names)
            print 'keypoints_names', keypoints_names
            del keypt_locs
            del keypt_names
            break
            
  
raw_input("Finished processing point clouds. Press enter to save key points to numpy file")
np.save(kpf + '_keypoints.npy', keypoints_locations)
np.save(kpf + '_keypoints_names.npy', keypoints_names)


#raw_input("press enter to add new entry to yaml file")

#entry = get_new_demo_entry_text(i, hole1_tracker[i], hole2_tracker[i], tcut_tracker[i], mcut_tracker[i], bcut_tracker[i], needle_tracker[i])
#add_new_entry_to_yaml_file(IROS_DATA_DIR, str(SEGNUM))

raw_input("press enter to finish")
