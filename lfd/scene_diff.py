import os.path as osp

DATA_DIR = osp.join(osp.dirname(__file__), "data")

# demo_clouds and exp_clouds are lists of point clouds, where the point clouds are a list of points
def get_scene_dist(demo_clouds, exp_clouds):
    #TODO: fill this in
    raise NotImplementedError

def find_argmin(d):
    min_key = None
    min_value = float("Inf")
    for k, v in d.items():
        if v < min_value:
            min_key = k
            min_value = v
    return min_key

# verb_info is a list of tuples with the demo_name and demo_info
# exp_clouds is the list of point clouds for the new scene
def get_closest_demo(verb_data_accessor, verb, exp_clouds):
    verb_info = verb_data_accessor.get_verb_info(verb)
    demo_scene_dists = {}
    for name, info in verb_info:
        demo_clouds = []
        for stage_num in xrange(verb_data_accessor.get_num_stages(name)):
            verb_stage_data = verb_data_accessor.get_demo_data(name)
            demo_clouds.append(verb_stage_data["object_cloud"][info.item]["xyz"])
        demo_scene_dists[name] = get_scene_dist(demo_clouds, exp_clouds)
    return find_argmin(demo_scene_dists)
