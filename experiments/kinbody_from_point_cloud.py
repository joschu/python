import openravepy
import numpy as np
env = openravepy.Environment()


##### Spheres ####
if False:
    body = openravepy.RaveCreateKinBody(env,'')
    body.SetName("cup")
    point_cloud = np.random.randn(30,3)
    radii = np.ones(len(point_cloud)) * .003
    body.InitFromSpheres(np.c_[point_cloud, radii],True)
    env.Add(body)

#### Convex hull mesh #####
if True:
    delaunay = scipy.spatial.Delaunay(point_cloud)
    body2 = openravepy.RaveCreateKinBody(env,'')
    body2.SetName("cup2")
    body2.InitFromTrimesh(openravepy.TriMesh(delaunay.vertices, delaunay.convex_hull.flatten()))
    env.Add(body2)


