import lfd
import yaml
from utils_lfd import group_to_dict
import os.path as osp
import h5py

class VerbStageInfo:
    def __init__(self, stage_name, verb, item, arms_used, special_point):
        self.stage_name = stage_name
        self.verb = verb
        self.item = item
        self.arms_used = arms_used
        self.special_point = special_point

class VerbDataAccessor:

    VERBS_H5 = "verbs2.h5"
    DEMO_YAML_FILE = "multi_item_verb_demos2.yaml"

    def __init__(self, test_info_dir=""):
        if test_info_dir == "":
            self.load_data("data")
        else:
            self.load_data(test_info_dir)

    # loads data from the specified directory
    def load_data(self, data_dir):
        self.h5path = osp.join(osp.dirname(lfd.__file__), data_dir, self.VERBS_H5)
        with open(osp.join(osp.dirname(lfd.__file__), data_dir, self.DEMO_YAML_FILE), "r") as fh:
            self.all_demo_info = yaml.load(fh)
            if self.all_demo_info is None:
                self.all_demo_info = {} 

    # returns the number of stages for a demo
    def get_num_stages(self, demo_name):
        for demo_name, demo_info in self.all_demo_info.items():
            if demo_name.find(demo_name) == 0:
                return len(demo_info["items"])
        return 0

    # returns the number of stages for a verb (assumes that all demos for each verb have the same number of stages)
    def get_num_stages_for_verb(self, verb):
        for name, info in self.all_demo_info.items():
            if info["verb"] == verb:
                return len(info["items"])
        raise KeyError("Could not find a demo corresponding to %s" % verb)

    def get_all_demo_info(self):
        return self.all_demo_info

    # returns the dictionary of information for a given demo
    def get_demo_info(self, demo_name):
        return self.all_demo_info[demo_name]

    def get_verb_info(self, verb):
        return [(name,info) for (name,info) in self.all_demo_info.items() if info["verb"] == verb]

    def get_verb_from_demo_name(self, demo_name):
        return self.get_demo_info(demo_name)["verb"]

    # returns a VerbStageInfo with information for a certain stage for a demo
    def get_stage_info(self, demo_name, stage_num):
        return self.get_stage_info_from_demo_info(self.get_demo_info(demo_name), stage_num)

    # returns a VerbStageInfo with information for a certain stage for a demo
    def get_stage_info_from_demo_info(self, demo_info, stage_num):
        if stage_num < 0:
            return None
        special_point = None if demo_info["special_pts"][stage_num] == "None" else demo_info["special_pts"][stage_num]
        return VerbStageInfo(demo_info["stages"][stage_num],
                             demo_info["verb"],
                             demo_info["items"][stage_num],
                             demo_info["arms_used"][stage_num],
                             special_point)

    # returns data for a certain stage
    def get_demo_stage_data(self, stage_name):
        h5file = h5py.File(self.h5path, "r")
        out = group_to_dict(h5file[stage_name])
        h5file.close()
        return out
