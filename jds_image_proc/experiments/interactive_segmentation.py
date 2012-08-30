import cv2
import numpy as np
from glob import glob
import jds_image_proc.smart_labeler as sl
import os
from os.path import join

def get_labels(bgr,mask):
    dirname = "/tmp/interactive_segmentation"
    bgrname = join(dirname,"bgr.png")
    maskname = join(dirname,"mask.png")
    if not os.path.exists(dirname): os.mkdir(dirname)
    cv2.imwrite(bgrname,bgr)
    cv2.imwrite(maskname,mask)
    ml = sl.MultiLabelerWithClassifier([bgrname],maskname)
    while not ml.wantsExit():
        ml.step()
    return ml.labels[0]

if __name__ == "__main__":
    bgr = cv2.imread("/home/joschu/Data/comm_towel2/images/data000000000000.pcd.jpg")
    mask = cv2.imread("/home/joschu/Data/comm_towel2/once/roi_mask.png")
    labels = get_labels(bgr,mask)
    cv2.imshow('hi',labels * 100)
    while True:
        cv2.waitKey(10)
