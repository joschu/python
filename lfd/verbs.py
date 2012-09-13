import lfd
import yaml
from utils_lfd import group_to_dict
import os.path as osp
import h5py




h5path = osp.join(osp.dirname(lfd.__file__),"data","verbs2.h5")
with open(osp.join(osp.dirname(lfd.__file__), "data", "verb_demos2.yaml"), "r") as fh:
    all_demo_info = yaml.load(fh)
    if all_demo_info is None: all_demo_info = {} 
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
def get_demo_data(demo_name):    
    h5file = h5py.File(h5path, "r")
    out =  group_to_dict(h5file[demo_name])
    h5file.close()
    return out
