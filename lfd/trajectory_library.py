from __future__ import division
import lfd
import numpy as np
from os.path import exists,join,dirname
import h5py

"""
Database layout
/demos (group)
   /0
   /1
   /2
   ...
/segments (group)
   /0
   /1
   /2
   ...
"""

class TrajectoryLibrary(object):
    
    library_dir = join(dirname(lfd.__file__),"data")
    
    
    def __init__(self, dbname, mode):
        if mode == "read": filemode = "r"
        elif  mode == "write": 
            if exists(join(self.library_dir, dbname)): filemode = "r+"
            else: filemode = "w"
        else:
            raise Exception("mode must be 'read' or 'write'")
        
        self.root = h5py.File(join(self.library_dir, dbname),filemode)
        if "segments" not in self.root: self.root.create_group("segments")
        if "demos" not in self.root: self.root.create_group("demos")
        
    def create_segments(self):
        "break demos into segments"
        if "segments" in self.root and len(self.root["segments"]) > 0: 
            del self.root["segments"]
            self.root.create_group("segments")
        for (name,traj) in self.root["demos"].items():
            break_times = get_break_times(traj)
            start_times = np.r_[0,break_times].astype('int')
            end_times = np.r_[break_times, len(traj)].astype('int')
            for (i,(start, end)) in enumerate(zip(start_times, end_times)):
                self.root["segments"].create_dataset("%s.%i"%(name, i), data=traj[start:end])
    
    def add_demo(self, array, name=None):

        if name is None:
            n_prev_demos = len(self.root["demos"].keys())            
            name = "%i"%n_prev_demos

        self.root["demos"][name] = array
    
    
def connected_components_1d(array):
    ccs = []
    last_a = array[0]
    if last_a: cur_cc = [0]        
    else: cur_cc = []
    
    for i in xrange(1,len(array)):
        if array[i]:
            cur_cc.append(i)
        elif last_a:
            ccs.append(cur_cc)
            cur_cc = []
        last_a = array[i]
            
    if len(cur_cc) > 0: ccs.append(cur_cc)
    return ccs
            
def get_break_times(traj):
    "midpoints of successive changes in contact states"
    both_open = (traj["grab_l"] == -1) & (traj["grab_r"] == -1)
    both_open_ccs = connected_components_1d(both_open)
    
    if len(both_open_ccs)==1: return [both_open_ccs[0][0] + both_open_ccs[0][-1]//2]
    
    if both_open[0]: both_open_ccs.pop(0)
    if both_open[-1]: both_open_ccs.pop(-1)
    return [(cc[0] + cc[-1])//2 for cc in both_open_ccs]
    # if they both start out or end open, don't break those interval
    
    
def get_change_inds(arr):
    "return set of i such that arr[i] != arr[i-1]"
    return np.flatnonzero(arr[1:] != arr[:-1])+1

def interactive_select_demo(library):
    # plot all of them in a matplotlib or opencv window
    # get kb input
    import matplotlib.pyplot as plt
    plt.ion()
    segment_group = library.root["segments"]
    
    n_segs = len(segment_group.keys())
    n_rows = int(round(np.sqrt(n_segs)))
    n_cols = int(np.ceil(n_segs / n_rows))    
    
    for (i,(name, data)) in enumerate(segment_group.items()):
        plt.subplot(n_rows, n_cols, i+1)
        if "object_points" in segment_group:
            points_n3 = data["object_points"][0]
        else:
            print "rope field is deprecated. use object points"
            points_n3 = data["rope"][0]
        plt.plot(points_n3[:,1], points_n3[:,0],'.')
        plt.title(name)
        
    plt.draw()
    while True:
        response = raw_input("type name of segment to select it, or type 'stop':\n?> ")
        if response == "stop": return response
        elif response in segment_group: return response
        else: print "invalid response. try again."
            
    