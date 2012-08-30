import rospy
import cv2
from brett2.ros_utils import xyzrgb2pc, pc2xyzrgb, get_tf_listener
import sensor_msgs.msg as sm
import scipy.ndimage as ndi
import numpy as np
import scipy.spatial.distance as ssd
import rospy
from jds_utils.func_utils import once


if rospy.get_name() == "/unnamed": 
    print "init node"
    rospy.init_node("red_dot_calib",disable_signals=True)
cv2.namedWindow("bgr",0)
cv2.startWindowThread()



def get_center():
    pc = rospy.wait_for_message("/camera/depth_registered/points", sm.PointCloud2)  
    xyz, bgr = pc2xyzrgb(pc)
    bgr = bgr.copy()
    hsv = cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    h=hsv[:,:,0]
    s=hsv[:,:,1]
    v=hsv[:,:,2]
    
    mask = ((h<10) | (h>150)) & (s > 100) & (v > 100)
    labels,max_label = ndi.label(mask)
    centers = []
    for label in xrange(1, max_label+1):
        us, vs = np.nonzero(labels == label)
        if len(us) >= 8:
            pts = xyz[us, vs, :]
            centers.append(pts.mean(axis=0))
    centers= np.array(centers)
    
    if len(centers) > 0:
        listener.waitForTransform(pc.header.frame_id, "l_gripper_tool_frame", rospy.Time(0), rospy.Duration(1))    
        gripper_trans,_ = listener.lookupTransform(pc.header.frame_id, "l_gripper_tool_frame", rospy.Time(0))
        gripper_trans = np.array(gripper_trans)
        dists = ssd.cdist(centers, gripper_trans[None,:])
        closest_center = centers[dists.argmin()]
        if dists.min() < .025:
            print "successfully got point"
            return gripper_trans, closest_center
        else:
            print "fail"
            return None
            
    
    contours = cv2.findContours(mask.astype('uint8'),cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)[0]
    cv2.drawContours(bgr,contours,-1,(0,255,0))
    cv2.imshow("bgr",bgr)
    cv2.waitKey(10)
    
    
    
positions = np.load("posititions.npy")
from brett2.PR2 import PR2
pr2 = PR2.create()
listener = pr2.tf_listener
fk_pos, pc_pos = [],[]
for pos in np.concatenate((positions,positions[::-1]),0):
    pr2.larm.goto_joint_positions(pos)
    pr2.join_all()
    rospy.sleep(2)
    maybe_pair = get_center() or get_center() or get_center()
    if maybe_pair is not None:
        fk_pos.append(maybe_pair[0])
        pc_pos.append(maybe_pair[1])
pcp1 = np.c_[pc_pos, np.ones((len(pc_pos),1))]
q=np.linalg.lstsq(pcp1, fk_pos)[0]

print q
