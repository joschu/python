import cv2
import numpy as np
import os,json
from collections import namedtuple
from utils import dir_tools
from image_proc import cv_drawing, utils_images
import scipy.ndimage as ndi

KeyFunc = namedtuple("KeyFunc","func desc")

class ImageWindow(object):

    def __init__(self):
        self._wantsExit = False
        self.image = None
        self.windowName = "image"
        cv2.namedWindow(self.windowName,cv2.cv.CV_WINDOW_NORMAL)
        self.keyFuncs = {}
        self.addKeyFunc(self.printHelp, 'h', 'print help')
        self.addKeyFunc(self.quit, 'q', 'quit')
    def addKeyFunc(self, fn, key, desc):
        kf = KeyFunc(fn,desc)
        self.keyFuncs[key] = kf
    def printHelp(self):
        print
        for (key,kf) in self.keyFuncs.items():
            print key,":",kf.desc
    def step(self):
        if self.image is not None:
            cv2.imshow(self.windowName,self.image)
            keycode = cv2.waitKey(10)
            if keycode != -1:
                try: 
                    char = chr(keycode)
                    self.processKey(char)
                except ValueError:
                    pass
    def processKey(self, key):
        if key in self.keyFuncs:
            self.keyFuncs[key].func()
    def quit(self):
        self._wantsExit = True
    def wantsExit(self):
        return self._wantsExit
            


class MultiLabeler(object):
    def __init__(self, imageFiles, labelFiles=None):
        self.imageFiles = imageFiles
        self.images =  [cv2.imread(imageFile) for imageFile in imageFiles]
        self.nImages = len(self.images)
        if labelFiles is None:
            self.labels = [np.zeros(shape=self.images[0].shape[:2], dtype='uint8') for _ in self.images]
        else: self.labels = [cv2.imread(labelFile,0) for labelFile in labelFiles]
        self.labelWindow = LabelWindow()
        self.ind = 0
        self.labelWindow.addKeyFunc(self.fwd, 'f', 'forward to next image')
        self.labelWindow.addKeyFunc(self.back, 'b', 'backward to prev image')
        self.labelWindow.addKeyFunc(self.clear, 'x', 'clear all labels on this image')
        self.labelWindow.setup(self.images[self.ind],self.labels[self.ind])

    def writeFiles(self, outDir):
        dir_tools.mkdir_ask(outDir)
        for (i,image) in enumerate(self.images):
            outFile = "%s.label.png"%os.path.basename(self.imageFiles[i])
            cv2.imwrite(os.path.join(outDir,outFile),self.labels[i])
            with open(os.path.join(outDir,"label_info.json"),"w") as fh:
                json.dump(
                    dict([(i, "LABEL_DESCRIPTION") for i in xrange(self.labelWindow.maxLabel)]),
                    fh)
        print "wrote files to",outDir

    def fwd(self):
        self.ind = min(self.ind+1,self.nImages-1)
        self.labelWindow.setup(self.images[self.ind],self.labels[self.ind])
    def back(self):
        self.ind = max(self.ind-1,0)
        self.labelWindow.setup(self.images[self.ind],self.labels[self.ind])
    def clear(self):
        self.labels[self.ind][:] = 0
        self.labelWindow.setup(self.images[self.ind],self.labels[self.ind])

    def step(self):
        self.labelWindow.step()
    def wantsExit(self):
        return self.labelWindow.wantsExit()


class LabelWindow(ImageWindow):

    def __init__(self):
        ImageWindow.__init__(self)
        self.maxLabel = 0
        self.currentLabel = 0
        self.brushSize = 5
        self.addKeyFunc(self.incSize,'+', 'increase brush size')
        self.addKeyFunc(self.decSize,'-', 'decrease brush size')
        self.addKeyFunc(self.setLabelInt,'s', 'set label index (0-9)')
        self.addKeyFunc(self.printLabelInfo, 'l', 'print label info')
        self.addKeyFunc(self.toggleColorView, 't', 'toggle colors bgr<->label')
        self.addKeyFunc(self.updateDisplay, 'u', 'update displayed image')
        self.addKeyFunc(self.removeBorders, 'p', 'remove borders between labeled regions')
        cv2.setMouseCallback(self.windowName, self.paint)

        self.label = None
        self.bgr = None
        self.labelImage = None
        self.displayMode = 0 # 0: labelImage. 1: bgr


    def setup(self, bgr, label):
        self.bgr = bgr
        self.label = label
        self.selectImage()

    def selectImage(self):
        if self.displayMode == 0: 
            self.updateLabelImage()
            self.image = self.labelImage
        elif self.displayMode == 1:
            self.updateBGR()
            self.image = self.bgrWithContours
    def updateLabelImage(self):
        self.labelImage = colorizeLabels(self.bgr, self.label)
        self.labelImage = cv_drawing.drawBorders(self.labelImage, self.label)
    def updateBGR(self):
        self.bgrWithContours = cv_drawing.drawBorders(self.bgr, self.label)
    def updateDisplay(self):
        if self.displayMode == 0: self.updateLabelImage()
        elif self.displayMode == 1: self.updateBGR()
        self.selectImage()
    def updateLabelPatch(self,x,y,r):
        labelImagePatch = self.labelImage[y-r:y+r+1,x-r:x+r+1,:]
        labelPatch = self.label[y-r:y+r+1,x-r:x+r+1]
        bgrPatch = self.bgr[y-r:y+r+1,x-r:x+r+1,:]
        labelImagePatch[:] = colorizeLabels(bgrPatch,labelPatch)         


    def paint(self,event, x, y, flags, param):
        if flags == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.label,(x,y),self.brushSize,self.currentLabel,thickness=-1)
            self.updateLabelPatch(x,y,self.brushSize)
    def toggleColorView(self):
        self.displayMode = 1-self.displayMode
        self.selectImage()
    def setLabelInt(self):
        # create number callback function
        # add number callbacks
        print "type new label (0-9)"
        keycode = cv2.waitKey(0)
        try: 
            self.currentLabel = int(chr(keycode))
            print "new label", self.currentLabel
        except ValueError: print "error: must be an number (0-9)"
        self.maxLabel = max(self.maxLabel, self.currentLabel)
    def incSize(self):
        self.brushSize += 1
        print "new brush size", self.brushSize
    def decSize(self):
        self.brushSize = max(self.brushSize-1, 1)
        print "new brush size", self.brushSize
    def printLabelInfo(self):
        print "%s    %s"%("label","hue")
        fmt = "%i    %i"
        for i in xrange(1,self.maxLabel+1):
            print fmt%(i, (i-1)*15)
    def removeBorders(self):
        print "what's the radius of the structuring element? (pixels)"
        keycode = cv2.waitKey(0)   
        try: 
            radius = int(chr(keycode))
        except ValueError: 
            print "error: must be an number (0-9)"        
            return
                
        sel = utils_images.disk(radius)
        for iLabel in xrange(1,self.maxLabel+1):
            self.label[(self.label == iLabel) & 
                       ndi.binary_dilation((self.label != iLabel) & (self.label > 0), sel)] = 0
        self.updateDisplay()


def colorizeLabels(bgr,label):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h,s = hsv[:,:,0], hsv[:,:,1]
    h[:] = (label-1)*15
    s[label==0] = 0
    s[label != 0] = 255
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)



