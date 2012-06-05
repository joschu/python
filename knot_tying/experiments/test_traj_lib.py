import h5py
import knot_tying.trajectory_library as tl
import knot_tying.rope_library as rl

library = rl.RopeTrajectoryLibrary("twohand_knot.h5","write")
library.create_segments()
some_rope = library.root["demos"]["demo0"]["rope"][10]
seg_name = library.lookup_closest(some_rope)
close_rope = library.root["segments"][seg_name]["rope"][0]
warped_traj = library.get_closest_and_warp(close_rope)