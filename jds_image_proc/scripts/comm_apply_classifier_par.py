#!/usr/bin/env python
from comm.mapper import ParallelMapper
from comm import comm
from jds_image_proc import annotation,classifiers
import cPickle
from os.path import join
import cv2

class ParLabelImages(ParallelMapper):
    def __init__(self):
        ParallelMapper.__init__(self, ["bmp"], 
                        [comm.MultiChannelImageMessage], 
                        "bmp", comm.SingleChannelImageMessage)
        

        self.mask = cv2.imread(self.args.mask,0)
            
    def add_extra_arguments(self, parser):
        ParallelMapper.add_extra_arguments(self,parser)
        parser.add_argument('classifier',type=str)
        parser.add_argument('--mask',default=join(comm.DATA_ROOT,"once/roi_mask.png"))
        return parser   
    
    def func(self, bgr):
        label = self.classifier.predict(bgr,self.mask)
        if self.worker_num == self.args.procs-1:
            cv2.imshow("labels (worker 0)",annotation.colorizeLabels(bgr,label))
            cv2.waitKey(5)           
        return label
    
    def worker_loop(self, worker_num):
        self.worker_num = worker_num
        with open(self.args.classifier,"r") as fh:
            print "unpickling"
            self.classifier = cPickle.load(fh)
            print "done"        
        ParallelMapper.worker_loop(self,worker_num)




comm.initComm()
M = ParLabelImages()
M.run()
