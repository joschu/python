import numpy as np
from utils import conversions


def transform_poses(xyzs,quats,F):
    x,y,z = xyzs.T
    xyzp = F.eval(x,y,z)
    jacs = F.grad(x,y,z)    
    
    new_quats = []
    for (quat,jac) in zip(quats,jacs):
        old_rot_mat = conversions.quat2mat(quat)
        new_rot_mat = np.dot(jac, old_rot_mat)
        q,r = np.linalg.qr(new_rot_mat.T)
        new_rot_mat_orth = np.sign(np.diag(r))[:,None]* q.T
        new_quat = conversions.mat2quat(new_rot_mat_orth)
        new_quats.append(new_quat)
    return xyzp, np.array(new_quats)

def mats2quats(mats):
    return np.array([conversions.mat2quat(mat) for mat in mats])
def quats2mats(quats):
    return np.array([conversions.quat2mat(quat) for quat in quats])

def transform_demo(reg, demo, left=True, right=True, rope=False):
    """
    reg: NonrigidRegistration object
    demo: array with the following fields: xyz_r, xyz_l, quat_r, quat_l, rope
    
    (todo: replace 'rope' with 'object_points')
    """
    
    warped_demo = np.asarray(demo).copy()
    
    if left:
        xyz_l_warped, rot_l_warped = reg.transform_poses(demo["xyz_l"], quats2mats(demo["quat_l"]))
        quat_l_warped = mats2quats(rot_l_warped)
        warped_demo["xyz_l"] = xyz_l_warped
        warped_demo["quat_l"] = quat_l_warped

    if right:
        xyz_r_warped, rot_r_warped = reg.transform_poses(demo["xyz_r"], quats2mats(demo["quat_r"]))
        quat_r_warped = mats2quats(rot_r_warped)
        warped_demo["xyz_r"] = xyz_r_warped
        warped_demo["quat_r"] = quat_r_warped
        
    if rope:
        old_rope_pts = demo["rope"].reshape(-1,3)
        new_rope_pts = reg.transform_points(old_rope_pts)
        warped_demo["rope"] = new_rope_pts.reshape(demo["rope"].shape)
        
        
        
    return warped_demo
    