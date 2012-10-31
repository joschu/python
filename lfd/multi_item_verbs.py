import lfd
import yaml
from utils_lfd import group_to_dict
import os.path as osp
import h5py

h5path = osp.join(osp.dirname(lfd.__file__),"data","verbs2.h5")
with open(osp.join(osp.dirname(lfd.__file__), "data", "multi_item_verb_demos2.yaml"), "r") as fh:
    all_demo_info = yaml.load(fh)
    if all_demo_info is None: all_demo_info = {} 


class VerbStageInfo:
    def __init__(self, stage_name, verb, item, arms_used, special_point):
        self.stage_name = stage_name
        self.verb = verb
        self.item = item
        self.arms_used = arms_used
        self.special_point = special_point

def get_num_stages(demo_base_name):
    for demo_name, demo_info in all_demo_info.items():
        if demo_name.find(demo_base_name) == 0:
            return len(demo_info["args"])
    return 0

def get_all_demo_info():
    return all_demo_info

def get_demo_info(demo_name):
    return all_demo_info[demo_name]

def get_verb_info(verb):
    return [(name,info) for (name,info) in all_demo_info.items() if info["verb"] == verb]

def get_closest_demo(verb, scene_info):
    verb_infos = get_verb_info(verb)
    if len(verb_infos) == 0: raise Exception("%s isn't in library"%verb)
    return get_verb_info(verb)[0] # xxx

def get_stage_info(demo_name, stage_num):
    return get_stage_info_from_demo_info(get_demo_info(demo_name), stage_num)

def get_closest_demo_verb_info_by_stage(verb, scene_info, stage_num):
    verb_infos = get_verb_info(verb)
    if len(verb_infos) == 0: raise Exception("%s isn't in library"%verb)
    # get_closest is not implemented yet
    closest_demo_name_info_pair = verb_infos[0]
    closest_demo_info = closest_demo_name_info_pair[1] #get the verb info out of the pair
    return get_stage_info_from_demo_info(closest_demo_info, stage_num)

def get_stage_info_from_demo_info(demo_info, stage_num):
    special_point = None if demo_info["special_pts"][stage_num] == "None" else demo_info["special_pts"][stage_num]
    return VerbStageInfo(demo_info["stages"][stage_num],
                         demo_info["verb"],
                         demo_info["args"][stage_num],
                         demo_info["arms_used"][stage_num],
                         special_point)
    

def get_demo_data(demo_name):
    h5file = h5py.File(h5path, "r")
    out = group_to_dict(h5file[demo_name])
    h5file.close()
    return out
