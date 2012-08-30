import cv2
import numpy as np
from jds_utils.func_utils import once

def drawContours(bgr,bw,color=(0,0,0)):
    bgr = bgr.copy()
    contours = cv2.findContours(bw.astype('uint8'),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[0]
    cv2.drawContours(bgr,contours,-1,color)
    return bgr

def drawBorders(bgr,bw,color=(0,0,0)):
    bgr = bgr.copy()
    border = np.zeros(bw.shape,dtype=bool)
    border[1:-1,1:-1] = ((bw[1:-1,1:-1] != bw[:-2,1:-1]) |
                         (bw[1:-1,1:-1] != bw[2:,1:-1]) |
                         (bw[1:-1,1:-1] != bw[1:-1,:-2]) |
                         (bw[1:-1,1:-1] != bw[1:-1,2:]))
    bgr[border] = color
    return bgr

@once
def get_font():
    return cv2.cv.InitFont(cv2.cv.CV_FONT_HERSHEY_COMPLEX,3, 3, 0.0, 5, cv2.cv.CV_AA)

def drawNums(arr,uvs,color=(0,255,0)):
    arr = arr.copy()
    get_font()
    uvs = np.array(uvs).astype('int32')
    for (i,(u,v)) in enumerate(uvs):
        cv2.putText(arr,str(i),(v,u),cv2.FONT_HERSHEY_PLAIN,
                    1,color)
    return arr