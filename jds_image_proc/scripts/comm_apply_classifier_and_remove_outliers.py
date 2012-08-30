#!/usr/bin/env python
from comm.mapper import Mapper
from comm import comm
import annotation,classifiers, utils_images
import cPickle
from os.path import join
import scipy.ndimage as ndi
import cv2
import numpy as np

class LabelImages(Mapper):
    def __init__(self):
        Mapper.__init__(self, ["pcd"], 
                        [comm.PointCloudMessage], 
                        "png", comm.SingleChannelImageMessage)
        
        self.window_name = "labeling"
        cv2.namedWindow(self.window_name,cv2.cv.CV_WINDOW_NORMAL)        
        with open(self.args.classifier,"r") as fh:
            self.classifier = cPickle.load(fh)
        self.mask = cv2.imread(self.args.mask,0)
            
    def add_extra_arguments(self, parser):
        parser.add_argument('classifier',type=str)
        parser.add_argument('--mask',default=join(comm.DATA_ROOT,"once/roi_mask.png"))
        return parser   
    
    def func(self, (xyz,bgr)):
        bgr = bgr.copy()
        label = self.classifier.predict(bgr,self.mask)
        x,y,z = xyz[:,:,0], xyz[:,:,1], xyz[:,:,2]
        depth = np.sqrt(x**2+y**2+z**2)
        depth[np.isnan(depth)] = np.inf
        good = np.zeros(depth.shape,bool)
        good[1:] |= np.abs(depth[1:] - depth[:-1]) < .02
        good[:,1:] |= np.abs(depth[:,1:] - depth[:,:-1]) < .02
        good = ndi.binary_erosion(good, np.ones((3,3)))
        label *= good        
        cv2.imshow(self.window_name,annotation.colorizeLabels(bgr,label))
        cv2.waitKey(10)
        return label
   
comm.setDataRoot()
M = LabelImages()
M.run()
