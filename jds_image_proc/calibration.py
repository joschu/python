from glob import glob
from os.path import join
import numpy as np
import json
from rave.openrave_kinematics import RaveRobot
import numpy as np
import pcl_utils
import cv2

def get_fk_positions(joints, tool_frame_offset, tool_frame = 'r_gripper_tool_frame'):
    "calculate position of point as calculated by forward kinematics"
    rr = RaveRobot()
    xyzs = []
    offset_homog = np.r_[tool_frame_offset,1]
    for joint in joints:
        rr.set_joint_positions(joint)
        transform = rr.robot.GetLink(tool_frame).GetTransform()
        xyzs.append(np.dot(transform), offset_homog)[:3]
    return xyzs


    