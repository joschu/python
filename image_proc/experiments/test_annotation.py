import cv2
import numpy as np
import annotation as ann
from glob import glob


def test_LabelWindow():

    images = glob("/home/joschu/Data/label_test/*7.jpg")

    bgr = cv2.imread(images[0])
    label = np.zeros(bgr.shape[:2],"uint8")
    lw = ann.LabelWindow()
    lw.setup(bgr,label)
    while not lw.wantsExit:
        lw.step()
    print "label \t pixel count"
    for (i,pixCount) in enumerate(np.bincount(lw.label.flatten())):
        print "%i \t %i"%(i, pixCount)

def test_MultiLabeler():
    imageFiles = glob("/home/joschu/Data/label_test/*.jpg")
    ml = ann.MultiLabeler(imageFiles)
    while not ml.wantsExit():
        ml.step()
    ml.writeFiles("/home/joschu/Data/label_test_ann")

if __name__ == "__main__":
    test_MultiLabeler()
