"""
Some functions for warping trajectories
"""


import numpy as np
from utils import conversions
from utils.conversions import quats2mats, mats2quats
from utils.math_utils import normalize

def draw_grid(rviz, f, mins, maxes, frame_id, xres = .1, yres = .1, zres = .04):
    grid_handles = []
    
    xmin, ymin, zmin = mins
    xmax, ymax, zmax = maxes

    ncoarse = 10
    nfine = 30
    xcoarse = np.arange(xmin, xmax, xres)
    ycoarse = np.arange(ymin, ymax, yres)
    zcoarse = np.arange(zmin, zmax, zres)
    xfine = np.linspace(xmin, xmax, nfine)
    yfine = np.linspace(ymin, ymax, nfine)
    zfine = np.linspace(zmin, zmax, nfine)
    
    lines = []
    
    for x in xcoarse:
        for y in ycoarse:
            xyz = np.zeros((nfine, 3))
            xyz[:,0] = x
            xyz[:,1] = y
            xyz[:,2] = zfine
            lines.append(f(xyz))

    for y in ycoarse:
        for z in zcoarse:
            xyz = np.zeros((nfine, 3))
            xyz[:,0] = xfine
            xyz[:,1] = y
            xyz[:,2] = z
            lines.append(f(xyz))
        
    for z in zcoarse:
        for x in xcoarse:
            xyz = np.zeros((nfine, 3))
            xyz[:,0] = x
            xyz[:,1] = yfine
            xyz[:,2] = z
            lines.append(f(xyz))

    for line in lines:
        grid_handles.append(rviz.draw_curve(conversions.array_to_pose_array(line, frame_id),width=.001,rgba=(1,1,0,1)))
                                
    return grid_handles


def interp_between(xnew, x0, x1, y0, y1):
    return ((x1 - xnew) * y0 + (xnew - x0) * y1) / (x1 - x0)

def calc_hand_pose(lpos, rpos, ori):
    ynew0 = normalize(lpos - rpos)
    xnew0 = normalize(ori[:,0] - np.dot(ynew0, ori[:,0]) * ynew0)
    znew0 = np.cross(xnew0, ynew0)
    tool_ori = np.c_[xnew0, ynew0, znew0]
    midpt = (lpos + rpos)/2
    new_open = np.linalg.norm(lpos - rpos) - 0.030284 # distance between frames when gripper closed
    midpt_to_tool = interp_between(new_open, .002,.087,  0.01179, 0.02581)
    #np.linalg.norm(((brett.robot.GetLink("r_gripper_r_finger_tip_link").GetTransform()+brett.robot.GetLink("r_gripper_l_finger_tip_link").GetTransform())/2 - brett.robot.GetLink("r_gripper_tool_frame").GetTransform())[:3,3])    
    tool_pos = tool_ori[:3,0] * midpt_to_tool + midpt
    new_pose = np.eye(4)
    new_pose[:3,:3] = tool_ori
    new_pose[:3,3] = tool_pos
    return new_pose, new_open


def group_to_dict(group):
    "deep copy hdf5 structure"
    out = {}
    for key in group.keys():
        if hasattr(group[key],"shape"):
            out[key] = np.asarray(group[key])
        else:
            out[key] = group_to_dict(group[key])
    return out


def transform_demo(reg, demo, left=True, right=True, cloud_xyz=False, object_clouds=False, l_offset = None, r_offset = None):
    """
    reg: NonrigidRegistration object
    demo: array with the following fields: r_gripper_xyzs, l_gripper_xyzs, r_gripper_quats, l_gripper_quats, cloud_xyz
    
    (todo: replace 'cloud_xyz' with 'object_points')
    """
    warped_demo = group_to_dict(demo)
        
    if left:
        if l_offset is None:
            l_tool_xyzs = demo["l_gripper_xyzs"]
        else:
            l_tool_xyzs = np.array([xyz + np.dot(rot[:3,:3], l_offset) for (xyz, rot) in zip(demo["l_gripper_xyzs"], quats2mats(demo["l_gripper_quats"]))])
        l_tool_xyzs_warped, rot_l_warped = reg.transform_frames(l_tool_xyzs, quats2mats(demo["l_gripper_quats"]))
        l_gripper_quats_warped = mats2quats(rot_l_warped)
        if l_offset is None:
            l_gripper_xyzs_warped = l_tool_xyzs_warped
        else:
            l_gripper_xyzs_warped = np.array([xyz - np.dot(rot[:3,:3], l_offset) for (xyz, rot) in zip(l_tool_xyzs_warped, rot_l_warped)])
        
        warped_demo["l_gripper_xyzs"] = l_gripper_xyzs_warped
        warped_demo["l_gripper_quats"] = l_gripper_quats_warped

    if right:
        if r_offset is None:
            r_tool_xyzs = demo["r_gripper_xyzs"]
        else:
            r_tool_xyzs = np.array([xyz + np.dot(rot[:3,:3], r_offset) for (xyz, rot) in zip(demo["r_gripper_xyzs"], quats2mats(demo["r_gripper_quats"]))])
        r_tool_xyzs_warped, rot_r_warped = reg.transform_frames(r_tool_xyzs, quats2mats(demo["r_gripper_quats"]))
        r_gripper_quats_warped = mats2quats(rot_r_warped)
        if r_offset is None:
            r_gripper_xyzs_warped = r_tool_xyzs_warped
        else:
            r_gripper_xyzs_warped = np.array([xyz - np.dot(rot[:3,:3], r_offset) for (xyz, rot) in zip(r_tool_xyzs_warped, rot_r_warped)])
        warped_demo["r_gripper_xyzs"] = r_gripper_xyzs_warped
        warped_demo["r_gripper_quats"] = r_gripper_quats_warped
        
    if cloud_xyz:
        old_cloud_xyz_pts = np.asarray(demo["cloud_xyz"]).reshape(-1,3)
        new_cloud_xyz_pts = reg.transform_points(old_cloud_xyz_pts)
        warped_demo["cloud_xyz"] = new_cloud_xyz_pts.reshape(demo["cloud_xyz"].shape)
        
    if object_clouds:
        for key in sorted(demo["object_clouds"].keys()):            
            old_cloud_xyz_pts = np.asarray(demo["object_clouds"][key]).reshape(-1,3)
            new_cloud_xyz_pts = reg.transform_points(old_cloud_xyz_pts)
            warped_demo["object_clouds"][key] = new_cloud_xyz_pts
        
    return warped_demo

def sorted_values(d):
    return [d[key] for key in sorted(d.keys())]

def transform_demo_with_fingertips(reg, demo, left=True, right=True, cloud_xyz=False):
    """
    reg: NonrigidRegistration object
    demo: array with the following fields: r_gripper_xyzs, l_gripper_xyzs, r_gripper_quats, l_gripper_quats, cloud_xyz
    
    (todo: replace 'cloud_xyz' with 'object_points')
    """
    
    warped_demo = dict([(key,np.asarray(value)) for (key, value) in demo.items()])
    
    if left:
        _, ori_warped = reg.transform_frames(demo["l_gripper_xyzs"], quats2mats(demo["l_gripper_quats"]))
        l_xyzs1 = reg.transform_points(demo["l_gripper_xyzs1"])
        l_xyzs2 = reg.transform_points(demo["l_gripper_xyzs2"])
        
        tool_xyzs = []
        tool_quats = []
        tool_angles = []
        
        for (pos1, pos2, ori) in zip(l_xyzs1, l_xyzs2, ori_warped):
            hmat, ang = calc_hand_pose(pos1, pos2, ori)
            xyz, quat = conversions.hmat_to_trans_rot(hmat)
            tool_xyzs.append(xyz)
            tool_quats.append(quat)
            tool_angles.append(ang)
        
        warped_demo["l_gripper_xyzs"] = tool_xyzs
        warped_demo["l_gripper_xyzs1"] = l_xyzs1
        warped_demo["l_gripper_xyzs2"] = l_xyzs2
        warped_demo["l_gripper_quats"] = tool_quats
        warped_demo["l_gripper_angles"] = tool_angles

    if right:
        _, ori_warped = reg.transform_frames(demo["r_gripper_xyzs"], quats2mats(demo["r_gripper_quats"]))
        r_xyzs1 = reg.transform_points(demo["r_gripper_xyzs1"])
        r_xyzs2 = reg.transform_points(demo["r_gripper_xyzs2"])
        
        tool_xyzs = []
        tool_quats = []
        tool_angles = []
        
        for (pos1, pos2, ori) in zip(r_xyzs1, r_xyzs2, ori_warped):
            hmat, ang = calc_hand_pose(pos1, pos2, ori)
            xyz, quat = conversions.hmat_to_trans_rot(hmat)
            tool_xyzs.append(xyz)
            tool_quats.append(quat)
            tool_angles.append(ang)
        
        warped_demo["r_gripper_xyzs"] = tool_xyzs
        warped_demo["r_gripper_xyzs1"] = r_xyzs1
        warped_demo["r_gripper_xyzs2"] = r_xyzs2        
        warped_demo["r_gripper_quats"] = tool_quats
        warped_demo["r_gripper_angles"] = tool_angles
        
    if cloud_xyz:
        old_cloud_xyz_pts = demo["cloud_xyz"].reshape(-1,3)
        new_cloud_xyz_pts = reg.transform_points(old_cloud_xyz_pts)
        warped_demo["cloud_xyz"] = new_cloud_xyz_pts.reshape(demo["cloud_xyz"].shape)
        
        
        
    return warped_demo
    