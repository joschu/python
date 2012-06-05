import image_proc.annotation as ann
import cv2
import numpy as np
import image_proc.classifiers as C
import cPickle, os
import utils.dir_tools

class MultiLabelerWithClassifier(ann.MultiLabeler):
    def __init__(self, imageFiles, maskFile, labelFiles = None, downsample=1, min_pix = 50):
        ann.MultiLabeler.__init__(self,imageFiles,labelFiles)
        self.mask = cv2.imread(maskFile,0)
        self.labelWindow.addKeyFunc(self.applyCls,'a','apply classifier to current image')
        self.labelWindow.addKeyFunc(self.trainCls,'r','train classifier')
        self.cls = C.LABLinearSVMWithClustering(ratio = 1./downsample, min_pix = min_pix)

    def applyCls(self):
        if self.cls.trained: 
            self.labels[self.ind] = self.cls.predict(self.images[self.ind],self.mask)
            self.labelWindow.setup(self.images[self.ind],self.labels[self.ind])
        else:
            print "need to train classifier first!"
            
    def trainCls(self):
        print "training..."
        changed_inds = [i for (i,label) in enumerate(self.labels) if label.any()]
        print changed_inds, "changed"
        self.cls.train([self.images[i] for i in changed_inds], [self.labels[i] for i in changed_inds])
        print "done"
        
    def writeFiles(self, outDir, writeLabels = True):
        dir_tools.mkdir_ask(outDir)
        if writeLabels: ann.MultiLabeler.writeFiles(self,outDir)
        with open(os.path.join(outDir,"classifier.pkl"),"w") as fh:
            cPickle.dump(self.cls, fh)
        np.savetxt(os.path.join(outDir,"coeffs.txt"),self.cls.classifier.coef_)
        np.savetxt(os.path.join(outDir,"intercepts.txt"),self.cls.classifier.intercept_)