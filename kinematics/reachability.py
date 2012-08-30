from __future__ import division
import kinematics
import kinematics.region_algebra as ra
from kinematics import sphere_sampling
from jds_utils import conversions
import h5py, os
import scipy.ndimage as ndi
import numpy as np
from jds_utils.yes_or_no import yes_or_no
from time import time
    
IKFAIL_COST = 1e3
COLLISION_COST = 1e6
    
def get_legal_base_positions(xticks, yticks, robot):
    env = robot.GetEnv()
    orig_transform = robot.GetTransform()
    good_mask = np.ones((xticks.size, yticks.size), bool)
    
    table = env.GetKinBody("table")
    assert table is not None
    bodies = [table]
    links = [robot.GetLink("l_shoulder_pan_link"), robot.GetLink("r_shoulder_pan_link")]
    
    with env:
        for (i,j) in np.ndindex(xticks.size, yticks.size):
            new_transform = orig_transform.copy()
            new_transform[0,3] = xticks[i]
            new_transform[1,3] = yticks[j]            
            robot.SetTransform(new_transform)
                        
            for link in links:
                for body in bodies:
                    good_mask[i,j] &= not env.CheckCollision(link, body)

    robot.SetTransform(orig_transform)
    return good_mask
    
def reach_cost(reachable):
    cost = np.empty(reachable.shape, float)
    
    reachable = reachable.astype('bool')
    reachable_region_cost = - ndi.distance_transform_cdt(reachable)
    
    np.putmask(cost,reachable,reachable_region_cost)
    np.putmask(cost,~reachable, IKFAIL_COST)
    return cost

def obstacle_cost(free):
    free = free.astype('bool')
    cost = np.empty(free.shape, float)
    
    free_region_cost = - ndi.distance_transform_cdt(free)
    
    np.putmask(cost,free,free_region_cost)
    np.putmask(cost,~free, COLLISION_COST)

    return cost
    
        
class ReachabilityDatabase(object):
    db_path = os.path.join(os.path.dirname(kinematics.__file__), "data", "reachability.h5")

    
    def __init__(self, mode):
        if mode == "read": filemode = "r"
        elif  mode == "write": 
            if os.path.exists(self.db_path): filemode = "r+"
            else: filemode = "w"
        else:
            raise Exception("mode must be 'read' or 'write'")
        
        self.h5file = h5py.File(self.db_path, filemode)
                
def get_reachable_region(db, manip, xyz, quat):
    manip_name = manip.GetName()
    group = db.h5file[manip_name]
        
    pointing_axis = conversions.quat2mat(quat)[:3,0]
    i_closest_axis = np.dot(group["pointing_axes"],pointing_axis).argmax()
    
    
    x,y,z = xyz
    xroot, yroot, zroot = manip.GetBase().GetTransform()[:3,3]
    
    zticks = np.asarray(group["zticks"])
    zind = int(round((z - zroot - zticks[0])/(zticks[1] - zticks[0])))
    
    if zind < 0 or zind >= zticks.size:
        raise Exception("z value too high")

    #zind = np.searchsorted(self.root["zticks"], z - zroot) + 1
    
    ee_mask = ra.Grid2(group["xticks"], group["yticks"], group[str(i_closest_axis)][:,:,zind])
    base_mask = ee_mask.flip().shift(x - xroot, y - yroot)
    return base_mask

def generate_db(db, manip, xlim, ylim, zlim, xres, yres, zres, subdivisions):
    manip_name = manip.GetName()
    group = db.h5file[manip_name]
            
    erase_all_data(group)
            
    pointing_axes = sphere_sampling.get_sphere_points(subdivisions)
    
    xticks = np.arange(xlim[0], xlim[1], xres)
    yticks = np.arange(ylim[0], ylim[1], yres)
    zticks = np.arange(zlim[0], zlim[1], zres)

    nx, ny, nz = xticks.size, yticks.size, zticks.size
    
    x0, y0, z0 = manip.GetBase().GetTransform()[:3,3]
    
    reachable = np.zeros((nx, ny, nz), dtype='uint8')
    
    t_start = time()
    count = 0        
    n_total_ik = len(pointing_axes) * reachable.size
    for (i_ax, x_ax) in enumerate(pointing_axes):
        
        for (ix, iy, iz) in np.ndindex(nx, ny, nz):
            if count % 200 == 100:
                t_remains = (n_total_ik - count) / count * (time() - t_start)
                print "%i hours, %i minutes, %i seconds remain"%(t_remains//3600, t_remains//60%60, t_remains%60)
            #if iz == 0:
                #print "orientation %i of %i. position %i of %i -----"%(i_ax+1, len(pointing_axes), count+1, reachable.size)
            posemat = np.eye(4)
            posemat[:3,3] = [xticks[ix] + x0, yticks[iy] + y0, zticks[iz] + z0]
            posemat[:3,:3] = make_orth_basis(x_ax)
            try:
                solns = manip.FindIKSolutions(posemat, 2+16)
                reachable[ix,iy,iz] = len(solns)
            except Exception:
                print "error when solving for ik"
                reachable[ix, iy, iz] = 0
            count += 1

        ax_str = str(i_ax)
        group[ax_str] = reachable
        
        
    group["xticks"] = xticks
    group["yticks"] = yticks
    group["zticks"] = zticks
    group["pointing_axes"] = pointing_axes


def create_distance_arrays(db):
    for manip_name in db.h5file:
        group = db.h5file[manip_name]
        for i in xrange(len(group["pointing_axes"])):
            array = np.asarray(group[str(i)])
            array = ndi.binary_erosion(array>0, np.ones((2,2,2)))
            inds_str = "%i_inds"%i
            dist_str = "%i_dist"%i
            erase_dataset(group, inds_str)
            erase_dataset(group, dist_str)
            dists, inds = ndi.distance_transform_edt(array==0, return_indices = True, return_distances = True)
            group[inds_str] = inds
            group[dist_str] = dists
            
    
def get_nearest_ik(db, manip, xyz, quat):
    manip_name = manip.GetName()
    group = db.h5file[manip_name]
        
    pointing_axis = conversions.quat2mat(quat)[:3,0]
    i_closest_axis = np.dot(group["pointing_axes"],pointing_axis).argmax()
    
    
    x,y,z = xyz
    xroot, yroot, zroot = manip.GetBase().GetTransform()[:3,3]
    
    xticks = np.asarray(group["xticks"])
    yticks = np.asarray(group["yticks"])
    zticks = np.asarray(group["zticks"])
    
    xind = int(round((x - xroot - xticks[0])/(xticks[1] - xticks[0])))
    yind = int(round((y - yroot - yticks[0])/(yticks[1] - yticks[0])))
    zind = int(round((z - zroot - zticks[0])/(zticks[1] - zticks[0])))
    #zind = np.searchsorted(self.root["zticks"], z - zroot) + 1

    xind = np.clip(xind, 0, xticks.size-1)
    yind = np.clip(yind, 0, yticks.size-1)
    zind = np.clip(zind, 0, zticks.size-1)

    ind_str = "%i_inds"%i_closest_axis
    i_x, i_y, i_z = group[ind_str][:,xind, yind, zind]

    assert group[str(i_closest_axis)][i_x, i_y, i_z] > 0
    x_feas = xticks[i_x] + xroot
    y_feas = yticks[i_y] + yroot
    z_feas = zticks[i_z] + zroot
    
    old_mat = conversions.quat2mat(quat)
    old_x, old_y, old_z = old_mat.T
    new_x = group["pointing_axes"][i_closest_axis]
    new_y = np.cross(old_z, new_x)
    new_z = np.cross(new_x, new_y)
    pose_mat = np.eye(4)
    pose_mat[:3,0] = new_x
    pose_mat[:3,1] = new_y / np.linalg.norm(new_y)
    pose_mat[:3,2] = new_z / np.linalg.norm(new_z)
    #pose_mat[:3,:3] = make_orth_basis(new_x)
    
    # XXX not sure why it currently sometimes fails without erosion
    # it always succeeds when I use make_orth_basis
    # solution should exist regardless of wrist rotation
    pose_mat[:3,3] = [x_feas, y_feas, z_feas]
    return manip.FindIKSolution(pose_mat, 2+16)
    
def plot_cost(grid):
    import matplotlib.pyplot as plt
    import scipy.stats as ss
    plt.clf()
    #plot_arr = ss.rankdata(grid.array).reshape(grid.array.shape)
    plot_arr = grid.array
    print grid.array.max(), grid.array.min()
    plt.imshow(plot_arr, extent = [grid.xticks[0], grid.xticks[-1], grid.yticks[0], grid.yticks[-1]])
    plt.title("cost")
    print "click on the plot to continue"
    plt.ginput()              
    
        
def create_mirror(db, from_name, to_name):

    if to_name not in db.h5file:
        db.h5file.create_group(to_name)
    from_group = db.h5file[from_name]

    to_group = db.h5file[to_name]
    erase_all_data(to_group)
                
    to_group["xticks"] = np.asarray(from_group["xticks"])
    to_group["yticks"] = -np.asarray(from_group["yticks"])[::-1]
    to_group["zticks"] = np.asarray(from_group["zticks"])
    to_group["pointing_axes"] = np.dot(from_group["pointing_axes"], np.diag([1,-1,1]))
    for i in xrange(len(from_group["pointing_axes"])):
        to_group[str(i)] = np.asarray(from_group[str(i)])
    
def find_min_cost_base(db, robot, manip_name, arm_poses, plotting = False):
            
    manip = robot.GetManipulator(manip_name)
    
    assert len(arm_poses) > 0
    total_cost = ra.Grid2AdditiveIdentity()
    for (xyz, quat) in arm_poses:        
        reachable_region = get_reachable_region(db, manip, xyz, quat)
        cost_for_pose = reach_cost(reachable_region.array)            
        cost = ra.Grid2(reachable_region.xticks, reachable_region.yticks, cost_for_pose, fill_value = IKFAIL_COST)
        total_cost += cost
        print "fill value",total_cost.fill_value
        if plotting: plot_cost(total_cost)      
        
    total_cost.array += obstacle_cost(get_legal_base_positions(total_cost.xticks, total_cost.yticks, robot))
    if plotting: plot_cost(total_cost)            
    
    xind, yind = np.unravel_index(cost.array.argmin(), cost.array.shape)
    print (total_cost.array < IKFAIL_COST/2).sum(),"good solutions"
    return (total_cost.xticks[xind], total_cost.yticks[yind]), total_cost.array[xind, yind]
        
def find_min_cost_base_bimanual(db, robot, left_poses, right_poses, plotting = False):
        
    leftarm = robot.GetManipulator("leftarm")
    rightarm = robot.GetManipulator("rightarm")

    cost = ra.Grid2AdditiveIdentity()
    for ((l_xyz, l_quat), (r_xyz, r_quat)) in zip(left_poses, right_poses):
        l_reachable_region = get_reachable_region(db, leftarm, l_xyz, l_quat)
        r_reachable_region = get_reachable_region(db, rightarm, r_xyz, r_quat)
        l_cost_for_pose = reach_cost(l_reachable_region.array)
        r_cost_for_pose = reach_cost(r_reachable_region.array)
        print "lmin",l_cost_for_pose.min(), "rmin", r_cost_for_pose.min()
        cost += ra.Grid2(l_reachable_region.xticks, l_reachable_region.yticks, l_cost_for_pose, fill_value=IKFAIL_COST)
        if plotting: plot_cost(cost)
        cost += ra.Grid2(r_reachable_region.xticks, r_reachable_region.yticks, r_cost_for_pose, fill_value=IKFAIL_COST)        
        if plotting: plot_cost(cost)
        
    cost.array += obstacle_cost(get_legal_base_positions(cost.xticks, cost.yticks, robot))
    if plotting: plot_cost(cost)            

    print (cost.array < IKFAIL_COST/2).sum(),"good solutions"

    xind, yind = np.unravel_index(cost.array.argmin(), cost.array.shape)
    return (cost.xticks[xind], cost.yticks[yind]), cost.array[xind,yind]

def erase_dataset(group, name):            
    if name in group: del group[name]
def erase_all_data(group):
    consent = yes_or_no("erase all data in %s?"%group.name)
    if consent:
        for name in group.keys(): erase_dataset(group, name)
    else:
        raise IOError
            
def make_orth_basis(x_ax):
    x_ax = x_ax / np.linalg.norm(x_ax)
    if np.allclose(x_ax, [1,0,0]): 
        return np.eye(3)
    elif np.allclose(x_ax, [-1, 0, 0]):
        return np.array([
            [-1, 0, 0],
            [0, -1, 0],
            [0, 0, 1]])
    else:
        y_ax = np.r_[0, x_ax[2], -x_ax[1]]
        y_ax /= np.linalg.norm(y_ax)
        z_ax = np.cross(x_ax, y_ax)
        return np.c_[x_ax, y_ax, z_ax]
