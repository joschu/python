import IPython
import trajoptpy
import openravepy as rave

env = rave.Environment()
env.Load("robots/pr2-beta-static.zae")
viewer = trajoptpy.GetViewer(env)
IPython.lib.inputhook.set_inputhook(viewer.Step)
raw_input("hi")
