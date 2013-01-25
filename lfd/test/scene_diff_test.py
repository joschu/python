from jds_utils.colorize import colorize
from lfd import scene_diff
from lfd import multi_item_verbs
import os.path as osp

TEST_DATA_DIR = "multi_item/scene_diff_data"

def report(x, msg=""):
    if x:
        print colorize("Pass", "green")
    else:
        print colorize("Fail: %s" % (msg), "red")

def test_scene_diff(verb, exp_name, correct_closest_name, data_dir):
    verb_data_accessor = multi_item_verbs.VerbDataAccessor(test_info_dir=osp.join("test", TEST_DATA_DIR, data_dir))
    exp_clouds = scene_diff.get_clouds_for_demo(verb_data_accessor, exp_name)
    scene_diff_closest_name, scene_diff_dists = scene_diff.get_closest_demo(verb_data_accessor, verb, exp_clouds, ignore=[exp_name], return_dists=True)
    if correct_closest_name == scene_diff_closest_name:
        report(True)
    else:
        if scene_diff_dists.has_key(correct_closest_name):
            msg = "Closest demo was %s at distance %f; demo %s had distance %f" % \
                  (scene_diff_closest_name, scene_diff_dists[scene_diff_closest_name],
                  correct_closest_name, scene_diff_dists[correct_closest_name])
        else:
            msg = "Closest demo was %s at distance %f; demo %s was not found" % \
                  (scene_diff_closest_name, scene_diff_dists[scene_diff_closest_name],
                  correct_closest_name)
        report(False, msg)

if __name__ == "__main__":
    test_scene_diff("grab", "grab-marker00", "grab-marker450", "grab_marker_l")
    #test_scene_diff("grab", "grab-marker900", "grab-marker450", "grab_marker_l")
    #test_scene_diff("place", "place-spoon-mediumgreen0", "place-spoon-mediumgreen1", "place_spoon_cup_l_l")
