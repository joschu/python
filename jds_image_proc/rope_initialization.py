import cv2
from jds_image_proc import skeletons
import jds_image_proc.mlabraw_jds_image_processing as mip
import scipy.spatial.distance as ssd
import scipy.ndimage as ndi
import networkx as nx, numpy as np
from jds_image_proc.pcl_utils import to3d, xyz2uv
from jds_image_proc import curves
import jds_utils.math_utils as mu
from jds_image_proc import cv_drawing
from comm import comm

N_SEGS = 100

def nearest_neighbor(x, ys):
    return mu.norms(ys - x[None,:],1).argmin()
def get_cc_centers(mask,n,min_pix=2):
    labels, max_label = ndi.label(mask)

    if max_label < n: raise Exception("could not find %i regions"%n)
    
    counts = np.bincount(labels.flatten())
    good_labels = np.flatnonzero(counts >= min_pix)[1:]
    good_labels_sorted = good_labels[counts[good_labels].argsort()[::-1]]
    good_labels_sorted = good_labels_sorted[:n]
    print good_labels_sorted
    
    out = np.array(ndi.center_of_mass(mask,labels,good_labels_sorted))
    print out
    return out
    
def plot_colorskel(paths2d,bgr,start_end, starts_ends):
    colorskel = np.zeros(bgr.shape,'uint8')
    for (i,path) in enumerate(paths2d):
        us,vs = np.array(path).T
        colorskel[us,vs] = np.random.randn(3)*255
    colorskel = cv2.resize(colorskel,(640*2,480*2))
    colorskel = cv_drawing.drawNums(colorskel, np.array(start_end)*2,(255,0,0))
    colorskel = cv_drawing.drawNums(colorskel, np.array(starts_ends)*2, (0,0,255))
    cv2.imshow('colorskel',colorskel)
    cv2.waitKey(10)

def initialize_rope(label_img, xyz,bgr, plotting=False):
    rope_mask = (label_img==1) | ndi.morphology.binary_dilation(label_img==2,np.ones((5,5)))
    rope_mask = ndi.binary_opening(rope_mask, np.ones((3,3)))
    
    if plotting: 
        cv2.imshow('bgr',bgr.copy())
        cv2.imshow('labels',label_img.astype('uint8')*50)
        cv2.waitKey(5)
    
    
    start_end = (uv_start, uv_end) = get_cc_centers(label_img==2,2)
    
    skel = mip.skeletonize(rope_mask)
    skel = skeletons.removeSpurs1(skel,10)
    
    if plotting: cv2.imshow('skel',skel*100)
    
    paths2d = skeletons.get_paths_2d(skel)
    endpts2d = []
    for path in paths2d:
        endpts2d.append(path[0])
        endpts2d.append(path[-1])
    i_start = nearest_neighbor(uv_start, endpts2d)
    i_end = nearest_neighbor(uv_end, endpts2d)
    print "start,end",i_start,i_end
    
    paths3d = [to3d(path,xyz) for path in paths2d]
    
    if plotting:
        starts_ends = []
        for path in paths2d:
            starts_ends.append(path[0])
            starts_ends.append(path[-1])
        plot_colorskel(paths2d,bgr,start_end,starts_ends)
    
    C = skeletons.make_cost_matrix(paths3d)
    PG = skeletons.make_path_graph(C, [len(path) for path in paths3d])
    (_,nodes) = skeletons.longest_path_between(PG,i_start, set([]), i_end)
    if nodes is None: raise Exception("longest path failed")
    
    print nodes
    total_path = []
    for node in nodes[::2]:
        if node%2 == 0: 
            total_path.extend(paths3d[node//2])
        else:
            total_path.extend(paths3d[node//2][::-1])
    total_path = np.array(total_path)        
            
    xyzs_unif = curves.unif_resample(total_path,N_SEGS+1,tol=.0025)

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
        for (i,path) in enumerate(paths3d):
            mlab.text3d(path[0,0],path[0,1],path[0,2],str(2*i),scale=.01,color=(0,0,0))
            mlab.text3d(path[-1,0],path[-1,1],path[-1,2],str(2*i+1),scale=.01,color=(0,0,0))
        mlab.show()
        
    
    return xyzs_unif, labels



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
        raise NotImplemented
        
