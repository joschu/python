import cv2
import classifiers as C
from glob import glob
import utils_images

if __name__ == "__main__":
    Cls = C.LABLinearSVMWithClustering(ratio=1)
    images = [cv2.imread(f) for f in sorted(glob("/home/joschu/Data/imgproc_bench/*.jpg"))]
    labels = [cv2.imread(f)[:,:,0].copy() for f in sorted(glob("/home/joschu/Data/imgproc_bench/labels/*.png"))]
    mask = cv2.imread("/home/joschu/Data/imgproc_bench/roi_mask.png")[:,:,0].copy()
    Cls.train(images,labels)
    predLabels0 = Cls.predict(images[-1],mask)
    print utils_images.confusionMat(labels[-1],predLabels0)[1:,1:]