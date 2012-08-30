#!/usr/bin/env python
from comm.mapper import Mapper
from comm import comm
import scipy.ndimage as ndi
from jds_image_proc.ransac import ransac, ConstantModel
import numpy as np
import cv_drawing
import cv2
import pcl_utils

def remove_bad_rows(x):
    good_rows = np.isfinite(x).all(axis=1)
    return x[good_rows]

class GetRegionCenters(Mapper):
    def __init__(self):
        Mapper.__init__(self, ["png", "pcd"], 
                        [comm.SingleChannelImageMessage, comm.PointCloudMessage], 
                        "txt", comm.ArrayMessage)
        self.window_name = "centers"
        cv2.namedWindow(self.window_name,cv2.cv.CV_WINDOW_NORMAL)
    def func(self, label_image, xyz_bgr):
        xyz_image, bgr = xyz_bgr

        mask = (label_image == self.args.label).copy()
        labels, max_label = ndi.label(mask)
        
        if (max_label == 0): return []
        
        counts = np.bincount(labels.flatten())
        good_labels = np.flatnonzero(counts >= self.args.min_pix)[1:]
        good_labels_sorted = good_labels[counts[good_labels].argsort()[::-1]]
        good_labels_sorted = good_labels_sorted[:self.args.max_blobs]

        centers = []

        for i in good_labels_sorted:
            mask = labels == i
            xyzs_blob = remove_bad_rows(xyz_image[mask])
            if len(xyzs_blob)==0: continue
            model,_,_ = ransac(xyzs_blob, ConstantModel, minDataPts=1, nIter=10, threshold=.04, nCloseRequired=self.args.min_pix)
            if model is not None:
                centers.append(model.mean)

        centers = np.array(centers).reshape(-1,3)
        plot_image = cv_drawing.drawNums(bgr,pcl_utils.xyz2uv(centers))
        cv2.imshow(self.window_name, plot_image)
        if self.args.pause:
            cv2.waitKey(0)
        else:
            cv2.waitKey(5)
        return centers

    def add_extra_arguments(self, parser):
        parser.add_argument('label',type=int)
        parser.add_argument('--min_pix',type=int,default=10,nargs='?')
        parser.add_argument('--max_blobs',type=int,default=10,nargs='?')
        parser.add_argument('--pause',action="store_true",default=False)
        return parser


    
comm.initComm()
M = GetRegionCenters()
M.run()
