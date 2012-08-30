#!/usr/bin/env python
from comm.mapper import Mapper
from comm import comm

class PCD2JPG(Mapper):
    def __init__(self):
        Mapper.__init__(self, ["pcd"], 
                        [comm.PointCloudMessage], 
                        "jpg", comm.MultiChannelImageMessage)
    def func(self, xyz_bgr):
        _, bgr = xyz_bgr
        return bgr
   
comm.initComm()
M = PCD2JPG()
M.run()
