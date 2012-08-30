import cv2
from cv2 import cv
from scikits.learn import svm
import scipy.ndimage as ndi
import numpy as np
import random
from jds_image_proc.mlabraw_jds_image_processing import remove_holes
from scipy.cluster import vq

def extractLabeled(img,labels):
    return img[labels>0]

def multIntPair(pair,x):
    return (int(pair[0]*x), int(pair[1]*x))

def resizeByRatio(image, ratio, interpType):
    return cv2.resize(image,
                      multIntPair(image.shape[1::-1],ratio),
                      interpolation=interpType)

class PixelClassifier(object):
    def __init__(self, ratio = 1):
        self.ratio = ratio
        self.trained = False

    def predict(self,image,mask=None):
        if not self.trained: raise Exception("need to train classifier first")
        imRS = resizeByRatio(image, self.ratio, cv.CV_INTER_LINEAR)
        maskRS = resizeByRatio(mask, self.ratio, cv.CV_INTER_NN) if mask is not None\
            else None
        xRS = self.preprocess(imRS)
        yRS = self.classify(xRS,maskRS)
        labelRS = self.postprocess(yRS)
        label = cv2.resize(labelRS,
                           image.shape[1::-1],
                           interpolation=cv.CV_INTER_NN)
        return label

    def train(self,images,labels,erosionRadius=0):

        if erosionRadius == 0: erodedLabels = labels
        else: erodedLabels = [ndi.binary_erosion(label,disk(erosionRadius)) for label in labels]

        yImages = [resizeByRatio(label, self.ratio, cv.CV_INTER_NN) for label in erodedLabels]
        imagesRS = [resizeByRatio(image, self.ratio, cv.CV_INTER_LINEAR) for image in images]

        xImages = [self.preprocess(image) for image in imagesRS]
        self.nFeat = xImages[0].shape[-1]

        xMats = [image[label>0] for (image,label) in zip(xImages,yImages)]
        yMats = [label[label>0] for label in yImages]

        self.trainMat(np.concatenate(xMats), np.concatenate(yMats))
        self.trained = True


    def classify(self,imageRS,maskRS):




        if maskRS is None: maskRS = np.ones(imageRS.shape[:2],dtype=bool)

        xMat = imageRS[maskRS.astype('bool')]
        yMat = self.classifyMat(xMat)

        yImageRS = np.zeros(imageRS.shape[:2], dtype='uint8')
        yImageRS[maskRS.astype('bool')] = yMat

        return yImageRS

    def preprocess(self,image):
        raise # e.g. cvtColor
    def classifyMat(self,x):
        raise # rows are observations
    def trainMat(self,x,y):
        raise
    def postprocess(self,label):
        raise # e.g. bwareaopen

def disk(r):
    xs,ys = np.mgrid[-r:r+1,-r:r+1]
    return xs**2 + ys**2 <= r**2

#def remove_holes(labels, min_size):
    #labels_noborders = labels.copy()
    #labels_noborders[1:-1,1:-1] = 
    #max_label = labels.max()
    
    #for label in xrange(1,max_label+1):
        #ndi.distance_transform_cdt(labels==label,return_indices=True)
        #D

class PixelLinearSVM(PixelClassifier):
    "classifier where the underlying classifier comes from sklearn"
    def trainMat(self, x, y, **params):
        y = y-1
        self.classifier = svm.LinearSVC(**params)
        subset = np.array(random.sample(range(len(x)), min(len(x),5000)))
        oldsize = 0
        for i in xrange(5):
            print "iter %i, subset size %i"%(i,len(subset))
            self.classifier.fit(x[subset],y[subset])
            yInf = self.classifier.predict(x)
            subset = np.unique(np.concatenate((subset,np.flatnonzero(yInf != y))))
            
            if oldsize/len(subset) > .9: break
            oldsize = len(subset)
            
        
    def classifyMat(self, x):
        return self.classifier.predict(x)+1

class LABLinearSVM(PixelLinearSVM):
    def __init__(self, ratio=1):
        PixelLinearSVM.__init__(self,ratio)

    def preprocess(self, bgr):
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)/256.
        return lab
    
    def postprocess(self, label):
        return remove_holes(label,100)


class PixelLinearSVMWithClustering(PixelClassifier):
    def __init__(self, ratio=1):
        PixelClassifier.__init__(self,ratio)


    def trainMat(self, x, y, **params):
        y = y-1
        k=4
        yNew, oldFromNew = makeBackgroundModel(x,y,k)
        # weights = [1]*y.max() + [1./k]*k
        # label2weight = dict([(i,w) for (i,w) in enumerate(weights)])
        label2weight = 'auto'
        
        self.classifier = svm.LinearSVC(**params)
        subset = np.array(random.sample(range(len(x)), min(len(x),5000)))
        oldsize = 0
        for i in xrange(5):
            print "iter %i, subset size %i"%(i,len(subset))
            self.classifier.fit(x[subset],yNew[subset],class_weight=label2weight)
            yInf = self.classifier.predict(x)
            subset = np.unique(np.concatenate((subset,np.flatnonzero(oldFromNew[yInf] != oldFromNew[y]))))
            
            if oldsize/len(subset) > .9: break
            oldsize = len(subset)
            
        self.oldFromNew = oldFromNew
            
    def classifyMat(self, x):
        return self.oldFromNew[self.classifier.predict(x)]+1            
            
            
class LABLinearSVMWithClustering(PixelLinearSVMWithClustering):
    def __init__(self, ratio=1, min_pix = 25):
        PixelLinearSVMWithClustering.__init__(self,ratio)
        self.min_pix = min_pix

    def preprocess(self, bgr):
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)/256.
        return np.concatenate([lab, lab**2], 2)

    def postprocess(self, label):
        return remove_holes(label,self.min_pix)            
            
def makeBackgroundModel(x,y,kBG):
    bgLabel= y.max()
    xBG = x[y==bgLabel]
    print "clustering..."
    _,clu = vq.kmeans2(xBG,kBG)
    print "done"
    yNew = y.copy()
    yNew[y==bgLabel] += clu
    oldFromNew = np.array(range(0,bgLabel) + kBG*[bgLabel])
    return yNew,oldFromNew
