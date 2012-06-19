import numpy as np
from utils import conversions
from utils.conversions import quats2mats, mats2quats

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

def transform_demo(reg, demo, left=True, right=True, cloud_xyz=False):
    """
    reg: NonrigidRegistration object
    demo: array with the following fields: r_gripper_xyzs, l_gripper_xyzs, r_gripper_quats, l_gripper_quats, cloud_xyz
    
    (todo: replace 'cloud_xyz' with 'object_points')
    """
    
    warped_demo = dict([(key,np.asarray(value)) for (key, value) in demo.items()])
    
    if left:
        l_gripper_xyzs_warped, rot_l_warped = reg.transform_frames(demo["l_gripper_xyzs"], quats2mats(demo["l_gripper_quats"]))
        l_gripper_quats_warped = mats2quats(rot_l_warped)
        warped_demo["l_gripper_xyzs"] = l_gripper_xyzs_warped
        warped_demo["l_gripper_quats"] = l_gripper_quats_warped

    if right:
        r_gripper_xyzs_warped, rot_r_warped = reg.transform_frames(demo["r_gripper_xyzs"], quats2mats(demo["r_gripper_quats"]))
        r_gripper_quats_warped = mats2quats(rot_r_warped)
        warped_demo["r_gripper_xyzs"] = r_gripper_xyzs_warped
        warped_demo["r_gripper_quats"] = r_gripper_quats_warped
        
    if cloud_xyz:
        old_cloud_xyz_pts = demo["cloud_xyz"].reshape(-1,3)
        new_cloud_xyz_pts = reg.transform_points(old_cloud_xyz_pts)
        warped_demo["cloud_xyz"] = new_cloud_xyz_pts.reshape(demo["cloud_xyz"].shape)
        
        
        
    return warped_demo
    