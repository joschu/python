import sensor_msgs.msg as sm
import rospy
import brett2
from brett2.PR2 import PR2
import os.path as osp
import numpy as np

rospy.init_node("calibrate_gripper_closed")


pr2 = PR2()
pr2.rgrip.set_angle(-1)
pr2.lgrip.set_angle(-1)
pr2.join_all()
rospy.sleep(1)
data_dir = osp.join(osp.dirname(brett2.__file__),"data")
with open(osp.join(data_dir,"l_closed_val"),"w") as fh:
    fh.write(str(pr2.lgrip.get_angle()))
with open(osp.join(data_dir,"r_closed_val"),"w") as fh:
    fh.write(str(pr2.rgrip.get_angle()))
