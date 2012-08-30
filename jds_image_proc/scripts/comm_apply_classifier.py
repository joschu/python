#!/usr/bin/env python
from comm.mapper import Mapper
from comm import comm
from jds_image_proc import annotation,classifiers
import cPickle
from os.path import join
import cv2

class LabelImages(Mapper):
    def __init__(self):
        Mapper.__init__(self, ["bmp"], 
                        [comm.MultiChannelImageMessage], 
                        "bmp", comm.SingleChannelImageMessage)
        
        self.window_name = "labeling"
        cv2.namedWindow(self.window_name,cv2.cv.CV_WINDOW_NORMAL)        
        with open(self.args.classifier,"r") as fh:
            print "unpickling"
            self.classifier = cPickle.load(fh)
            print "done"
        self.mask = cv2.imread(self.args.mask,0)
            
    def add_extra_arguments(self, parser):
        parser.add_argument('classifier',type=str)
        parser.add_argument('--mask',default=join(comm.DATA_ROOT,"once/roi_mask.png"))
        return parser   
    
    def func(self, bgr):
        label = self.classifier.predict(bgr,self.mask)
        cv2.imshow(self.window_name,annotation.colorizeLabels(bgr,label))
        cv2.waitKey(5)
        return label

comm.initComm()
M = LabelImages()
M.run()
