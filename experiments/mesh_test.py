import numpy as np
import cloudprocpy
cloud = cloudprocpy.readPCDXYZ("/home/joschu/Proj/trajoptrave/bigdata/laser_cloud.pcd")
cloud = cloudprocpy.downsampleCloud(cloud, .01)
cloud = cloudprocpy.boxFilter(cloud, .5,5,-3,3,.1,2.5)
cloud.save("/tmp/cloud.ply")
xyzorig = cloud.to2dArray()[:,:3]
xyzn = cloudprocpy.mlsAddNormals(cloud).to2dArray()
xyz = xyzn[:,:3]
normals = xyzn[:,4:7]

from trajoptpy import math_utils
normals = math_utils.normr(normals)
camerapos = np.array([0,0,1.3])
ndotp = ((camerapos[None,:] - xyz) * normals).sum(axis=1)
normals *= np.sign(ndotp)[:,None]

import trajoptpy, openravepy
env = openravepy.RaveGetEnvironment(1)
if env is None:
    env = openravepy.Environment()
viewer = trajoptpy.GetViewer(env)
import IPython
IPython.lib.inputhook.set_inputhook(viewer.Step)

xyzn[:,4:7] = normals
cloudwn = cloudprocpy.CloudXYZN()
cloudwn.from2dArray(xyzn)
cloudwn.save("/tmp/lasercloud.ply")

handles = []
handles.append(env.plot3(xyzorig,4,[0,1,0]))
linelist = np.empty((len(xyz)*2, 3))
linelist[::2,:] = xyz
linelist[1::2,:] = xyz+normals*.075
handles.append(env.drawlinelist(linelist,1))
#mesh = cloudprocpy.createMesh(cloudwn, "MarchingCubes")
handles.append(env.drawarrow([0,0,0],camerapos,.006))
#handles.append(env.drawtrimesh(mesh.getCloud().to2dArray()[:,:3], np.array(mesh.getFaces())))