import cv2
import numpy as np
from glob import glob
import smart_labeler as sl


def test_SmartLabeler():
    imageFiles = glob("/home/joschu/Data/comm_towel/jpgs/*.jpg")

    maskFile = "/home/joschu/Data/comm_towel/once/roi_mask.png"


    ml = sl.MultiLabelerWithClassifier(imageFiles, maskFile)
    while not ml.wantsExit():
        ml.step()
    ml.writeFiles("/home/joschu/Data/label_test_ann")

if __name__ == "__main__":
    test_SmartLabeler()
