from comm.vision_pipeline import PIPELINE
from comm import pipeline
from comm import comm
import os
import matplotlib.pyplot as plt

comm.initComm()
os.chdir(comm.DATA_ROOT)
plot = pipeline.PipelinePlot(PIPELINE.restrict_to_target("human_towel"))
plot.loop()

