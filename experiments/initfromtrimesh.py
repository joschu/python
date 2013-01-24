import openravepy
import numpy as np
import scipy.spatial
env = openravepy.Environment()
point_cloud = np.random.randn(12,3)

gi = openravepy.KinBody.GeometryInfo()
gi._meshcollision = openravepy.Trimesh

body = openravepy.RaveCreateKinBody(env,'')
body.SetName("cup")

def calc_hull(points):
    if len(points) == 1:
        return points, []
    else:
        delaunay = scipy.spatial.Delaunay(point_cloud)
        return delaunay.points, delaunay.convex_hull
    

delaunay = scipy.spatial.Delaunay(point_cloud)
body.InitFromTrimesh(openravepy.TriMesh(delaunay.points, delaunay.convex_hull), True)
env.Add(body)

print delaunay.points
print delaunay.convex_hull
print len(delaunay.points)

#import trajoptpy
#cc = trajoptpy.GetCollisionChecker(env)
#cc.AllVsAll()
