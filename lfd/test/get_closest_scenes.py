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

    verb = verb_data_accessor.get_verb_from_demo_name(demo_name)
    exp_clouds = scene_diff.get_clouds_for_demo(verb_data_accessor, demo_name)
    scene_diff_closest_name = scene_diff.get_closest_demo(verb_data_accessor, verb, exp_clouds, ignore=[demo_name])
    return scene_diff_closest_name

# finds closest scenes for all demos for verb in data directory
def get_closest_scenes(data_dir, verb):
    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test_info_dir=osp.join("test", BASE_DATA_DIR, data_dir))
    demos_for_verb = verb_data_accessor.get_verb_info(verb)
    closest = {}
    for demo_name, info in demos_for_verb:
        exp_clouds = scene_diff.get_clouds_for_demo(verb_data_accessor, demo_name)
        scene_diff_closest_name = scene_diff.get_closest_demo(verb_data_accessor, verb, exp_clouds, ignore=[demo_name])
        closest[demo_name] = scene_diff_closest_name
    return closest

def print_usage():
    print "Arguments:"
    print "--dir: look at data in this directory (must always be specified)"
    print "--verb: find closest scenes for tasks with this verb" 
    print "--demo: find closest scene for this demo"

def results_for_data_dir_as_str(data_dir, closest):
    results = "%s:\n" % data_dir
    for demo_name, closest_demo_name in closest.items():
        results += "%s is closest to %s\n" % (demo_name, closest_demo_name)
    return results

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir")
    parser.add_argument("--verb")
    parser.add_argument("--demo")
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = get_args()
    if (sys.argv) == 0:
        print_usage()
    else:
        if args.verb: # find closest scenes for a certain test directory
            closest = get_closest_scenes(args.dir, args.verb)
            results = results_for_data_dir_as_str(args.dir, closest)
        elif args.demo: # find closest scenes for a single demo
            closest_name = get_closest_single_scene(args.dir, args.demo)
            results = "%s:\n%s is closest to %s\n" % (args.dir, args.demo, closest_name)

    print
    print "##### RESULTS #####"
    print
    print results
