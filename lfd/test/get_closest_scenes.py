import argparse
import sys
from lfd import multi_item_verbs, scene_diff
import os
import os.path as osp

# script that uses scene_diff to find the closest demo to each of the test demos

BASE_DATA_DIR = "multi_item/empty_move_data"

# finds the closest scene to a single demo
def get_closest_single_scene(data_dir, demo_name):
    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test_info_dir=osp.join("test", BASE_DATA_DIR, data_dir))
    all_demo_info = verb_data_accessor.get_all_demo_info()
    all_demo_names = all_demo_info.keys()
    assert demo_name in all_demo_names, "all_demo_names: %s" % (str(all_demo_names))

    verb = all_demo_info[all_demo_names[0]]["verb"]
    exp_clouds = scene_diff.get_clouds_for_demo(verb_data_accessor, demo_name)
    scene_diff_closest_name = scene_diff.get_closest_demo(verb_data_accessor, verb, exp_clouds, ignore=[demo_name])
    return scene_diff_closest_name

# finds closest scenes for all demos in data directory
def get_closest_scenes(data_dir):
    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test_info_dir=osp.join("test", BASE_DATA_DIR, data_dir))
    all_demo_info = verb_data_accessor.get_all_demo_info()
    all_demo_names = all_demo_info.keys()
    verb = all_demo_info[all_demo_names[0]]["verb"]
    closest = {}
    for demo_name in all_demo_names:
        exp_clouds = scene_diff.get_clouds_for_demo(verb_data_accessor, demo_name)
        scene_diff_closest_name = scene_diff.get_closest_demo(verb_data_accessor, verb, exp_clouds, ignore=[demo_name])
        closest[demo_name] = scene_diff_closest_name
    return closest

# get a list of all demo directories in the test data directory
def get_all_dirs():
    abs_base_dir = osp.join(osp.dirname(osp.abspath(__file__), BASE_DATA_DIR))
    all_dirs = [entry for entry in os.listdir(abs_base_dir) if osp.isdir(osp.join(abs_base_dir, entry))]
    return all_dirs

# finds closest scenes for all demos for all test data
def get_all_closest_scenes():
    all_closest = {}
    for data_dir in get_all_dirs():
        all_closest[data_dir] = get_closest_scenes(data_dir)
    return all_closest

def print_usage():
    print "Arguments:"
    print "'all': finds closest scenes for all data"
    print "<data_dir>: finds closest scenes for demos in data_dir"
    print "<data_dir> <demo_name>: finds closest scene for demo_name in data_dir"

def results_for_data_dir_as_str(data_dir, closest):
    results = "%s:\n" % data_dir
    for demo_name, closest_demo_name in closest.items():
        results += "%s is closest to %s\n" % (demo_name, closest_demo_name)
    return results

if __name__ == "__main__":
    if len(sys.argv) == 1:
        print_usage()
    elif len(sys.argv) == 2:
        if sys.argv[1] == "all": # find closest scenes for everything
            all_closest = get_all_closest_scenes()
            results = ""
            for data_dir, info in all_closest.items():
                results += results_for_data_dir_as_str(data_dir, info)
        else: # find closest scenes for a certain test directory
            data_dir = sys.argv[1]
            closest = get_closest_scenes(data_dir)
            results = results_for_data_dir_as_str(data_dir, closest)
    elif len(sys.argv) == 3: # find closest scenes for a single demo
        data_dir, demo_name = sys.argv[1], sys.argv[2]
        closest_name = get_closest_single_scene(data_dir, demo_name)
        results = "%s:\n%s is closest to %s\n" % (data_dir, demo_name, closest_name)

    print
    print "##### RESULTS #####"
    print
    print results
