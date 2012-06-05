from lfd import trajectory_library as tl


library= tl.TrajectoryLibrary("lm.h5", "read")
tl.interactive_select_demo(library)
