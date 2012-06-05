from comm.vision_pipeline import PIPELINE
from comm import pipeline
from comm import comm
import os

comm.initComm()
print comm.DATA_ROOT
#
#comm.resetDataDir()

os.chdir(comm.DATA_ROOT)
import subprocess
subprocess.check_call("rm -rf kinect labels images rope_pts towel_pts human_rope human_towel once/init_rope.txt",shell=True)

#pipeline.execute_series(PIPELINE, False)
pipeline.execute_parallel(PIPELINE.restrict_to_target("human_towel"), lifetime = 10, max_lag = 5)
