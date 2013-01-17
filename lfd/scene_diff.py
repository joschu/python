import os.path as osp

DATA_DIR = osp.join(osp.dirname(__file__), "data")

# demo_clouds and exp_clouds are lists of point clouds, where the point clouds are a list of points
def get_scene_dist(demo_clouds, exp_clouds):
    #TODO: fill this in
    return 0

def find_argmin(d):
    min_key = None
    min_value = float("Inf")
    for k, v in d.items():
        if v < min_value:
            min_key = k
            min_value = v
    return min_key

def get_clouds_for_demo(verb_data_accessor, demo_name):
    demo_clouds = []
    for stage_num in xrange(verb_data_accessor.get_num_stages(demo_name)):
        demo_stage_info = verb_data_accessor.get_stage_info(demo_name, stage_num)
        demo_stage_name = demo_name + "-%i"%stage_num
        demo_stage_data = verb_data_accessor.get_demo_data(demo_stage_name)
        demo_clouds.append(demo_stage_data["object_cloud"][demo_stage_info.item]["xyz"])
    return demo_clouds

# verb_info is a list of tuples with the demo_name and demo_info
# exp_clouds is the list of point clouds for the new scene
# ignore is a list of demos that shouldn't be returned
# if return_dists is True, then return the costs mapping (without the ignored demos)
def get_closest_demo(verb_data_accessor, verb, exp_clouds, ignore=[], return_dists=False):
    verb_info = verb_data_accessor.get_verb_info(verb)
    demo_scene_dists = {}
    for name, info in verb_info:
        demo_clouds = get_clouds_for_demo(verb_data_accessor, name)
        demo_scene_dists[name] = get_scene_dist(demo_clouds, exp_clouds)
    for ignored_demo in ignore:
        if demo_scene_dists.has_key(ignored_demo):
            demo_scene_dists.pop(ignored_demo)

    closest_demo = find_argmin(demo_scene_dists)

    if return_dists:
        return (closest_demo, demo_scene_dists)
    else:
        return closest_demo
