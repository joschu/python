from comm import comm
import networkx as nx
from image_proc.pcl_utils import to3d, xyz2uv
from image_proc import curves
import numpy as np, scipy.ndimage as ndi, itertools as it, scipy.spatial.distance as ssd
import cv2
from image_proc.utils_images import get_cc_centers
try: import image_proc.mlabraw_image_processing as mip
except Exception: pass


N_CTRL_PTS = 100
MAX_DIST=.03

def writeArray(arr, fh):
    arr = np.atleast_2d(arr)
    if arr.shape[0] == 1: arr = arr.T
    fh.write("%i %i\n"%(arr.shape[0], arr.shape[1]))
    np.savetxt(fh, arr, fmt='%.4g')

class RopeInitMessage(comm.Message):
    def writeDataTo(self, fname):
        xyz,l = self.data
        with open(fname,"w") as fh:
            writeArray(xyz,fh)
            writeArray(l.astype('uint8'), fh)
    def readDataFrom(self, fname):
        raise NotImplementedError


STEPS4 = [(1,-1),(1,0),(1,1),(0,1)]

def skel2graph(bwthin):
    G = nx.Graph()    
    us, vs = np.nonzero(bwthin)
    for (u,v) in zip(us,vs):
        for (du,dv) in STEPS4:
            if bwthin[u+du,v+dv]:
                G.add_edge((u,v),(u+du,v+dv))
    return G        

def longest_shortest_path(G):
    ends = [node for (node,deg) in G.degree().items() if deg == 1]

    best_length = 0
    best_path = None    
    for (end0,end1) in it.combinations(ends,2):
        path = nx.shortest_path(G,source=end0,target=end1)
        if path:
            length = len(path)

            if length > best_length:
                best_length = length
                best_path = path

    return best_path        

def initialize_rope(label_img, xyz,bgr, plotting=False):

    # XXX that sucks
    rope_mask = (label_img==1) | ndi.morphology.binary_dilation(label_img==2,np.ones((5,5)))
    rope_mask = ndi.binary_opening(rope_mask, np.ones((3,3)))
    rope_mask = ndi.binary_dilation(rope_mask, np.ones((15,15)))

    skel = mip.skeletonize(rope_mask)

    if plotting: 
        cv2.imshow('bgr',bgr.copy())
        cv2.imshow('labels',label_img.astype('uint8')*50)
        cv2.imshow("skel", skel.astype('uint8')*50)
        cv2.imshow("rope_mask", rope_mask.astype('uint8')*50)
        cv2.waitKey(5)


    #start_end = (uv_start, uv_end) = get_cc_centers(label_img==2,2)


    G = skel2graph(skel)
    path2d = longest_shortest_path(G)
    path3d = to3d(path2d,xyz)


    xyzs_unif = curves.unif_resample(path3d,N_CTRL_PTS,tol=.0025)

    #us,vs = xyz2uv(xyzs_unif).T
    #labels = label_img[us,vs]
    labels = np.ones(xyzs_unif.shape[0]-1,'int')
    labels[0] = labels[-1] = 2

    if plotting:
        xs,ys,zs = xyzs_unif.T
        us_skel, vs_skel = np.nonzero(skel)
        xs_skel, ys_skel, zs_skel = to3d(np.c_[us_skel, vs_skel], xyz).T

        import matplotlib.pyplot as plt, enthought.mayavi.mlab as mlab
        mlab.plot3d(xs,ys,zs,np.arange(len(xs)),tube_radius=.0025, colormap='spectral')
        mlab.points3d(xs_skel, ys_skel, zs_skel, scale_factor=.02, color=(1,1,1),opacity=.1)
        for (i,path) in enumerate(path3d):
            mlab.text3d(path[0,0],path[0,1],path[0,2],str(2*i),scale=.01,color=(0,0,0))
            mlab.text3d(path[-1,0],path[-1,1],path[-1,2],str(2*i+1),scale=.01,color=(0,0,0))
        mlab.show()


    return xyzs_unif, labels

def initialize_rope_from_cloud(xyzs, plotting=False):
    xyzs = xyzs.reshape(-1,3)
    if len(xyzs) > 500: xyzs = xyzs[::len(xyzs)//500]

    pdists = ssd.squareform(ssd.pdist(xyzs,'sqeuclidean'))
    G = nx.Graph()
    for (i_from, row) in enumerate(pdists):
        to_inds = np.flatnonzero(row[:i_from] < MAX_DIST**2)
        for i_to in to_inds:
            G.add_edge(i_from, i_to, weight = pdists[i_from, i_to])

    A = nx.floyd_warshall_numpy(G)
    A[np.isinf(A)] = 0
    (i_from_long, i_to_long) = np.unravel_index(A.argmax(), A.shape)

    nodes = G.nodes();
    path = nx.shortest_path(G, source=nodes[i_from_long], target=nodes[i_to_long])
    xyz_path = xyzs[path,:]
    xyzs_unif = curves.unif_resample(xyz_path,N_CTRL_PTS,tol=.005)
    labels = np.ones(len(xyzs_unif)-1,'int')
    labels[[0,-1]] = 2

    if plotting:
        import enthought.mayavi.mlab as mlab
        mlab.plot3d(*xyzs_unif.T, tube_radius=.001)
        mlab.points3d(*xyzs.T, scale_factor=.01)
        mlab.show()

    return xyzs_unif, labels
