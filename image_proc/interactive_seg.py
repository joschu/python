import cv2
import numpy as np

def get_table_height(xyz):
    z = xyz[:,:,2]
    zvals = z[np.isfinite(z)]
    zmin = zvals.min()
    zmax = zvals.max()
    bins = np.arange(zmin, zmax, .001)
    counts,_ = np.histogram(zvals, bins=bins)
    table_height = bins[counts.argmax()]
    return table_height

class GetClick:
    xy = None      
    done = False
    def callback(self,event, x, y, flags, param):
        if self.done:
            return
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.xy = (x,y)
            self.done = True
            
class GetMouse:
    xy = None
    done = False
    def callback(self, event, x, y, flags, param):
        if self.done:
            return
        elif event == cv2.EVENT_MOUSEMOVE:
            self.xy = (x,y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            self.xy = (x,y)
            self.done = True


def callback(self,event, x, y, flags, param):
    if self.done == True:
        pass
    elif event == cv2.EVENT_LBUTTONDOWN:
        self.xys.append((x,y))
    elif event == cv2.EVENT_MBUTTONDOWN:
        self.done = True

if __name__ == "__main__":
    npzfile = np.load("/home/joschu/python/lfd/data/images/open0.npz")
    xyz, rgb = npzfile['xyz'], npzfile['bgr']
    rgb = rgb.copy()
    gc = GetClick()
    cv2.namedWindow("rgb")
    cv2.setMouseCallback("rgb", gc.callback)
    while not gc.done:
        cv2.imshow("rgb", rgb)
        cv2.waitKey(10)
    # cv2.destroyWindow("rgb")
    gm = GetMouse()
    xy_br = np.array(gc.xy)
    print xy_br
    # cv2.namedWindow("rgb")
    cv2.setMouseCallback("rgb", gm.callback)
    while not gm.done:
        img = rgb.copy()
        #cv2.circle(img, gc.xy, 3, (255,0,0))

        if gm.xy is not None:
            xy_tl = np.array(gm.xy)
            cv2.rectangle(img, tuple(xy_br), tuple(xy_tl), (0,255, 0))
        cv2.imshow("rgb",img)
        cv2.waitKey(10)
        
    xy_tl, xy_br = sorted([xy_tl, xy_br],key = lambda x:x[0])
    #mask = np.zeros(xyz.shape[:2],dtype='uint8')
    #rectangle = gm.xy + tuple(2*(center - np.r_[gm.xy]))
    #tmp1 = np.zeros((1, 13 * 5))
    #tmp2 = np.zeros((1, 13 * 5))    
    #result = cv2.grabCut(rgb,mask,rectangle,tmp1, tmp2,10,mode=cv2.GC_INIT_WITH_RECT)
        
    #from cv2 import cv
    #mask[mask == cv.GC_BGD] = 0
    #mask[mask == cv.GC_PR_BGD] = 63
    #mask[mask == cv.GC_FGD] = 255  
    #mask[mask == cv.GC_PR_FGD] = 192
    #cv2.imshow('mask',mask)
    #cv2.waitKey(10)


    table_height = get_table_height(xyz)
    xl, yl = xy_tl
    w, h = xy_br - xy_tl
    mask = np.zeros((h,w),dtype='uint8')    
    mask.fill(cv2.GC_PR_BGD)
    mask[xyz[yl:yl+h, xl:xl+w,2] > table_height+.01] = cv2.GC_PR_FGD


    tmp1 = np.zeros((1, 13 * 5))
    tmp2 = np.zeros((1, 13 * 5))    
    cv2.grabCut(rgb[yl:yl+h, xl:xl+w, :],mask,(0,0,0,0),tmp1, tmp2,10,mode=cv2.GC_INIT_WITH_MASK)
    
    
    mask[mask == cv2.GC_BGD] = 0
    mask[mask == cv2.GC_PR_BGD] = 63
    mask[mask == cv2.GC_FGD] = 255  
    mask[mask == cv2.GC_PR_FGD] = 192
    
    contours = cv2.findContours(1-mask.astype('uint8')%2,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[0]
    cv2.drawContours(rgb[yl:yl+h, xl:xl+w, :],contours,-1,(0,255,0))
    
    cv2.imshow('mask',mask)
    cv2.imshow('rgb', rgb)
    cv2.waitKey(10)

    