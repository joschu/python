"""
Some functions for warping trajectories
"""


import numpy as np
from jds_utils import conversions
from jds_utils.conversions import quats2mats, mats2quats
from jds_utils.math_utils import normalize
from copy import deepcopy
from utils_lfd import group_to_dict

def draw_grid(rviz, f, mins, maxes, frame_id, xres = .1, yres = .1, zres = .04):
    grid_handles = []
    
    xmin, ymin, zmin = mins
    xmax, ymax, zmax = maxes

    ncoarse = 10
    nfine = 30
    xcoarse = np.arange(xmin, xmax, xres)
    ycoarse = np.arange(ymin, ymax, yres)
    if zres == -1: zcoarse = [(zmin+zmax)/2.]
    else: zcoarse = np.arange(zmin, zmax, zres)
    xfine = np.linspace(xmin, xmax, nfine)
    yfine = np.linspace(ymin, ymax, nfine)
    zfine = np.linspace(zmin, zmax, nfine)
    
    lines = []
    if len(zcoarse) > 1:    
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
        grid_handles.append(rviz.draw_curve(conversions.array_to_pose_array(line, frame_id),width=.0005,rgba=(1,1,0,1)))
                                
    return grid_handles


def interp_between(xnew, x0, x1, y0, y1):
    return ((x1 - xnew) * y0 + (xnew - x0) * y1) / (x1 - x0)

def calc_hand_pose(lpos, rpos, ori):
    """
    Do IK on gripper using fingertips
    lpos: left fingertip position
    rpos: right fingertip position
    ori: desired orientation. note that open/close direction is determined by lpos and rpos argument
    """
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



def transform_demo(reg, demo, left=True, right=True, cloud_xyz=False, object_clouds=False, l_offset = None, r_offset = None):
    """
    reg: NonrigidRegistration object
    demo: array with the following fields: r_gripper_xyzs, l_gripper_xyzs, r_gripper_quats, l_gripper_quats, cloud_xyz
    
    (todo: replace 'cloud_xyz' with 'object_points')
    """
    warped_demo = group_to_dict(demo)
        
    if left:
        if l_offset is None:
            l_tool_xyzs = demo["l_gripper_tool_frame"]["position"]
        else:
            l_tool_xyzs = np.array([xyz + np.dot(rot[:3,:3], l_offset) for (xyz, rot) in zip(demo["l_gripper_tool_frame"]["position"], quats2mats(demo["l_gripper_tool_frame"]["orientation"]))])
        l_tool_xyzs_warped, rot_l_warped = reg.transform_frames(l_tool_xyzs, quats2mats(demo["l_gripper_tool_frame"]["orientation"]))
        l_gripper_quats_warped = mats2quats(rot_l_warped)
        if l_offset is None:
            l_gripper_xyzs_warped = l_tool_xyzs_warped
        else:
            l_gripper_xyzs_warped = np.array([xyz - np.dot(rot[:3,:3], l_offset) for (xyz, rot) in zip(l_tool_xyzs_warped, rot_l_warped)])
        
        warped_demo["l_gripper_tool_frame"]["position"] = l_gripper_xyzs_warped
        warped_demo["l_gripper_tool_frame"]["orientation"] = l_gripper_quats_warped

    if right:
        if r_offset is None:
            r_tool_xyzs = demo["r_gripper_tool_frame"]["position"]
        else:
            r_tool_xyzs = np.array([xyz + np.dot(rot[:3,:3], r_offset) for (xyz, rot) in zip(demo["r_gripper_tool_frame"]["position"], quats2mats(demo["r_gripper_tool_frame"]["orientation"]))])
        r_tool_xyzs_warped, rot_r_warped = reg.transform_frames(r_tool_xyzs, quats2mats(demo["r_gripper_tool_frame"]["orientation"]))
        r_gripper_quats_warped = mats2quats(rot_r_warped)
        if r_offset is None:
            r_gripper_xyzs_warped = r_tool_xyzs_warped
        else:
            r_gripper_xyzs_warped = np.array([xyz - np.dot(rot[:3,:3], r_offset) for (xyz, rot) in zip(r_tool_xyzs_warped, rot_r_warped)])
        warped_demo["r_gripper_tool_frame"]["position"] = r_gripper_xyzs_warped
        warped_demo["r_gripper_tool_frame"]["orientation"] = r_gripper_quats_warped
        
    if cloud_xyz:
        old_cloud_xyz_pts = np.asarray(demo["cloud_xyz"]).reshape(-1,3)
        new_cloud_xyz_pts = reg.transform_points(old_cloud_xyz_pts)
        warped_demo["cloud_xyz"] = new_cloud_xyz_pts.reshape(demo["cloud_xyz"].shape)
        
    if object_clouds:
        for key in sorted(demo["object_clouds"].keys()):            
            old_cloud_xyz_pts = np.asarray(demo["object_clouds"][key]["xyz"]).reshape(-1,3)
            new_cloud_xyz_pts = reg.transform_points(old_cloud_xyz_pts)
            warped_demo["object_clouds"][key]["xyz"] = new_cloud_xyz_pts
        
    return warped_demo

def sorted_values(d):
    return [d[key] for key in sorted(d.keys())]

def transform_tracked_states(f, demo):
    assert 'tracked_states' in demo

    # f takes demo -> test
    # in practice, the initial tracked state might be slightly different from
    # the initial demo cloud, so we should fit g : tracked -> demo
    # (based on the initial demo cloud)
    # and then warp all tracked states with h = f . g

    import registration
    xyz_tracked_ds = np.reshape(demo['tracked_states'][0], (-1, 3))
    xyz_demo_ds = demo['cloud_xyz_ds']
    #g = registration.tps_rpm(xyz_tracked_ds, xyz_demo_ds, reg_init=1,reg_final=1,n_iter=100,verbose=False, plotting=False)
    #g = registration.ScaledRigid3d(); g.fit(xyz_tracked_ds, xyz_demo_ds)
    #S, g_A, g_b = registration.fit_affine_by_tpsrpm(xyz_tracked_ds, xyz_demo_ds); print 'got affine transformation', g_A, g_b
    g = registration.Rigid3d(); g.fit(xyz_tracked_ds, xyz_demo_ds)

    #TODO: this is rope-specific
    warped_tracked_states = []
    for s in demo['tracked_states']:
        # s is an array of control points [..., x_i, y_i, z_i, ...]
        pts = np.reshape(s, (-1, 3))
        pts = g.transform_points(pts)
#        pts = np.dot(pts, g_A.T) + g_b
        warped_pts = f.transform_points(pts)
        warped_tracked_states.append(warped_pts.reshape((1,-1))[0])
    print demo['tracked_states'][-1], warped_tracked_states[-1]
    #warped_demo['tracked_states'] = wts
    #warped_demo['tracked_states'] = demo['tracked_states']
    return warped_tracked_states

def transform_demo_with_fingertips(f, demo):
    """
    demo has a bunch of fields with arrays
    this function uses the Transformation f to transform some of the fields of it using f
    """    
    
    warped_demo = {}

    for lr in "lr":
        if "%s_gripper_tool_frame"%lr in demo:    
            _, ori = f.transform_frames(demo["%s_gripper_tool_frame"%lr]["position"], quats2mats(demo["l_gripper_tool_frame"]["orientation"]))
            xyz_fingertip0 = f.transform_points(demo["%s_gripper_l_finger_tip_link"%lr]["position"])
            xyz_fingertip1 = f.transform_points(demo["%s_gripper_r_finger_tip_link"%lr]["position"])
        
            tool_xyzs = []
            tool_quats = []
            tool_angles = []
        
            print calc_hand_pose(xyz_fingertip0[0], xyz_fingertip1[0], ori[0])[0][:3,:3]
            for (pos0, pos1, o) in zip(xyz_fingertip0, xyz_fingertip1, ori):
                hmat, ang = calc_hand_pose(pos0, pos1, o)
                xyz, quat = conversions.hmat_to_trans_rot(hmat)
                tool_xyzs.append(xyz)
                tool_quats.append(quat)
                tool_angles.append(ang)
        
            warped_demo["%s_gripper_tool_frame"%lr]={}        
            warped_demo["%s_gripper_l_finger_tip_link"%lr]={}        
            warped_demo["%s_gripper_l_finger_tip_link"%lr]["position"]=xyz_fingertip1
            warped_demo["%s_gripper_r_finger_tip_link"%lr]={}
            warped_demo["%s_gripper_r_finger_tip_link"%lr]["position"]=xyz_fingertip0
    
            warped_demo["%s_gripper_tool_frame"%lr]["position"] = np.array(tool_xyzs)
            warped_demo["%s_gripper_tool_frame"%lr]["orientation"] = np.array(tool_quats)
            warped_demo["%s_gripper_joint"%lr] = np.array(tool_angles)
        
        
    return warped_demo
    
    
def transform_verb_demo(warp, demo,l_offset = None, r_offset = None):
    """
    demo: array with the following fields: r_gripper_xyzs, l_gripper_xyzs, r_gripper_quats, l_gripper_quats, cloud_xyz, possible object_clouds
    transform each field using warp
    l_offset and r_offset tell you where the tools are, so you can do tool stuff
    (note: this doesn't currently work when important point is moved relative to demo)
    
    """
    warped_demo = group_to_dict(demo) # deep copy it
        
    if demo["arms_used"] in "lb":
        if l_offset is None:
            l_tool_xyzs = demo["l_gripper_tool_frame"]["position"]
        else:
            l_tool_xyzs = np.array([xyz + np.dot(rot[:3,:3], l_offset) for (xyz, rot) in zip(demo["l_gripper_tool_frame"]["position"], quats2mats(demo["l_gripper_tool_frame"]["orientation"]))])
        l_tool_xyzs_warped, rot_l_warped = warp.transform_frames(l_tool_xyzs, quats2mats(demo["l_gripper_tool_frame"]["orientation"]))
        l_gripper_quats_warped = mats2quats(rot_l_warped)
        if l_offset is None:
            l_gripper_xyzs_warped = l_tool_xyzs_warped
        else:
            l_gripper_xyzs_warped = np.array([xyz - np.dot(rot[:3,:3], l_offset) for (xyz, rot) in zip(l_tool_xyzs_warped, rot_l_warped)])
        
        warped_demo["l_gripper_tool_frame"]["position"] = l_gripper_xyzs_warped
        warped_demo["l_gripper_tool_frame"]["orientation"] = l_gripper_quats_warped

    if demo["arms_used"] in "rb":
        if r_offset is None:
            r_tool_xyzs = demo["r_gripper_tool_frame"]["position"]
        else:
            r_tool_xyzs = np.array([xyz + np.dot(rot[:3,:3], r_offset) for (xyz, rot) in zip(demo["r_gripper_tool_frame"]["position"], quats2mats(demo["r_gripper_tool_frame"]["orientation"]))])
        r_tool_xyzs_warped, rot_r_warped = warp.transform_frames(r_tool_xyzs, quats2mats(demo["r_gripper_tool_frame"]["orientation"]))
        r_gripper_quats_warped = mats2quats(rot_r_warped)
        if r_offset is None:
            r_gripper_xyzs_warped = r_tool_xyzs_warped
        else:
            r_gripper_xyzs_warped = np.array([xyz - np.dot(rot[:3,:3], r_offset) for (xyz, rot) in zip(r_tool_xyzs_warped, rot_r_warped)])
        warped_demo["r_gripper_tool_frame"]["position"] = r_gripper_xyzs_warped
        warped_demo["r_gripper_tool_frame"]["orientation"] = r_gripper_quats_warped
        
    if "object_clouds" in demo:
        for key in sorted(demo["object_clouds"].keys()):            
            old_cloud_xyz_pts = np.asarray(demo["object_clouds"][key]["xyz"]).reshape(-1,3)
            new_cloud_xyz_pts = warp.transform_points(old_cloud_xyz_pts)
            warped_demo["object_clouds"][key]["xyz"] = new_cloud_xyz_pts
        
    return warped_demo
