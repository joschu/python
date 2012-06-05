#!/usr/bin/env python
from comm.mapper import Mapper
from comm import comm

class PCD2BMP(Mapper):
    def __init__(self):
        Mapper.__init__(self, ["pcd"], 
                        [comm.PointCloudMessage], 
                        "bmp", comm.MultiChannelImageMessage)
    def func(self, xyz_bgr):
        _, bgr = xyz_bgr
        return bgr
   
comm.initComm()
M = PCD2BMP()
M.run()
