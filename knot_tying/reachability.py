import numpy as np
import skimage.graph as sig
import openravepy
import scipy.ndimage as ndi
import utils.math_utils as mu

INITED = False

DL=.025

def init(with_viewer = False):
    global PR2, ENV, LEFT, RIGHT, INITED, HANDLES, TABLE, WITH_VIEWER
    if not INITED:
        WITH_VIEWER = with_viewer
        ENV = openravepy.Environment()
        ENV.Load("/home/joschu/bulletsim/data/knots/scene.env.xml")
        PR2 = ENV.GetRobots()[0]
        LEFT = PR2.GetManipulator("leftarm")
        RIGHT = PR2.GetManipulator("rightarm")
        TABLE = ENV.GetKinBody("table")
        INITED = True
        if with_viewer:
            ENV.SetViewer('qtcoin')
        HANDLES = []
        

def make_left_db():
    init()
    
    xs = np.arange(.35,1.1,DL) #relative to torso lift link
    ys = np.arange(-.6,.75,DL)
    zs = np.arange(-.4,.4,DL)
    
    x_torso, y_torso, z_torso = PR2.GetLink("torso_lift_link").GetTransform()[:3,3]
    
    nx, ny, nz = xs.size, ys.size, zs.size
    
    reachable = np.empty((nx, ny, nz), 'uint8')
    
    
    for (i,j,k) in np.ndindex(nx, ny, nz):
        if k == 0: print (i,j,k)
        posemat = np.array([[-1, 0,  0,  xs[i] + x_torso],
                            [0, 1,  0,  ys[j] + y_torso],
                            [0, 0,  -1,  zs[k] + z_torso],
                            [0, 0,  0,  1]])
        solns = LEFT.FindIKSolutions(posemat, 2+8)
        reachable[i,j,k] = len(solns)
        print reachable[i,j,k],
        
    np.savez("/home/joschu/bulletsim/data/knots/l_reachability.npz", 
        reachable = reachable,
        xticks = xs,
        yticks = ys,
        zticks = zs)


def intround(x):
    return int(round(x))

def get_collision_mask(xticks, yticks):
    orig_transform = PR2.GetTransform()
    
    coll = np.empty((xticks.size, yticks.size), bool)
    with ENV:
        for (i,j) in np.ndindex(xticks.size, yticks.size):
            new_transform = orig_transform.copy()
            new_transform[0,3] = xticks[i]
            new_transform[1,3] = yticks[j]
            PR2.SetTransform(new_transform)
            coll[i,j] = ENV.CheckCollision(PR2.GetLink("l_shoulder_pan_link"), TABLE)\
                    | ENV.CheckCollision(PR2.GetLink("r_shoulder_pan_link"), TABLE)

    PR2.SetTransform(orig_transform)
    return coll
    
def get_feasible_path(stack):
    nt, nx, ny = stack.shape    
    
    mcp_obj = sig.MCP_Geometric(stack,
        [[1,0,0],
        [1,0,1],[1,0,-1],
        [1,1,0],[1,-1,0],
        [1,1,1],[1,1,-1],[1,-1,1],[1,-1,-1]])

    starts = [(0,i,j) for (i,j) in np.ndindex(nx, ny)]
    ends = [(nt-1,i,j) for (i,j) in np.ndindex(nx, ny)]
    costs, traceback = mcp_obj.find_costs(starts, ends)

    i = costs[-1].argmin()
    ix, iy = np.unravel_index(i, costs[-1].shape)

    print "cost:", costs[-1].min()
    #import matplotlib.pyplot as plt; plt.imshow(stack[0],extent = [yticks[0],yticks[-1],xticks[0],xticks[-1]],origin='lower')

    triples = mcp_obj.traceback((nt-1, ix, iy))    
    return np.array([(ix,iy) for (t,ix,iy) in triples])


def shift_and_place_image(image, xshift, yshift, xticksim, yticksim, xtickstarg, ytickstarg):
    xllind = intround( (xshift + xticksim[0] - xtickstarg[0])/DL )
    yllind = intround( (yshift + yticksim[0] - ytickstarg[0])/DL )
    
    nxim, nyim = xticksim.size, yticksim.size
    nxtarg, nytarg = xtickstarg.size, ytickstarg.size
    
    newimage = np.zeros((nxtarg, nytarg), image.dtype)
    newimage[xllind:xllind + nxim, yllind:yllind + nyim] = image
    
    return newimage
    
    
def get_xy_bounds(xyzs, xticksir, yticksir):
    xtargmax, ytargmax, _ = xyzs.max(axis=0)
    xtargmin, ytargmin, _ = xyzs.min(axis=0)
    
    xmin = xtargmin + xticksir[0]
    xmax = xtargmax + xticksir[-1]     
    ymin = ytargmin + yticksir[0]
    ymax = ytargmax + yticksir[-1]
    
    return [xmin, xmax, ymin, ymax]
    
    
def get_base_positions2(xyzs_world):
    xtorso, ytorso, ztorso = xyz_torso = PR2.GetLink("torso_lift_link").GetTransform()[:3,3] - PR2.GetLink("base_footprint").GetTransform()[:3,3]


    f = np.load("/home/joschu/bulletsim/data/knots/l_reachability.npz")

    invreach = f["reachable"][::-1, ::-1, :] #places that can reach the origin
    invreach = ndi.binary_erosion(invreach,np.ones((3,3)))

    xticksir = - f["xticks"][::-1]
    yticksir = - f["yticks"][::-1]
    zticksir = f["zticks"]

    [xmin, xmax, ymin, ymax] = get_xy_bounds(xyzs_world, xticksir, yticksir) # bounds for torso position array

    xticks = np.arange(xmin-DL, xmax+DL, DL) # torso positions
    yticks = np.arange(ymin-DL, ymax+DL, DL)

    base_costs = np.zeros((len(xyzs_world), xticks.size, yticks.size))
    for (i,(x, y, z)) in enumerate(xyzs_world):
        zind = intround(  (z - ztorso) / DL  )
        base_costs[i] = shift_and_place_image(1 + (invreach[:,:,zind]<=0)*1000, x, y, xticksir, yticksir, xticks, yticks)

    xinds_base, yinds_base = get_feasible_path(base_costs).T
    return np.c_[xticks[xinds_base] - xtorso, yticks[yinds_base] - ytorso]
    

def get_base_positions_bimanual(rars, joints):
    PR2.SetDOFValues(joints)
    f = np.load("/home/joschu/bulletsim/data/knots/l_reachability.npz")
    
    
    xtorso, ytorso, ztorso = xyz_torso = PR2.GetLink("torso_lift_link").GetTransform()[:3,3] - PR2.GetLink("base_footprint").GetTransform()[:3,3]
        
    xyz_l = rars["xyz_l"]
    xyz_r = rars["xyz_r"]
    
    left_used = (rars["grab_l"] > -1).any()
    right_used = (rars["grab_r"] > -1).any()
    print "left_used: %i, right_used: %i"%(left_used, right_used)
    
    invreachL = f["reachable"][::-1, ::-1, :] #places that can reach the origin
    invreachR = f["reachable"][::-1, :, :] #places that can reach the origin

    invreachL = ndi.binary_erosion(invreachL,np.ones((3,3,3)))
    invreachR = ndi.binary_erosion(invreachR,np.ones((3,3,3)))
    

    xticksir = - f["xticks"][::-1]
    yticksirL = - f["yticks"][::-1]
    yticksirR = f["yticks"]
    zticksir = f["zticks"]
    
    leftbounds = [xminL, xmaxL, yminL, ymaxL] = get_xy_bounds(xyz_l, xticksir, yticksirL) # bounds for torso position array
    rightbounds = [xminR, xmaxR, yminR, ymaxR] = get_xy_bounds(xyz_r, xticksir, yticksirR) 
    
    [xmin, xmax, ymin, ymax] = [min(xminL, xminR), max(xmaxL, xmaxR), min(yminL, yminR), max(ymaxL, ymaxR)]

    if WITH_VIEWER:
        HANDLES.append(ENV.drawlinestrip(points=np.array([[xmin, ymin, 0],
                                                        [xmin, ymax, 0],
                                                        [xmax, ymax, 0],
                                                        [xmax, ymin, 0]]),
                               linewidth=1.0))  

    xticks = np.arange(xmin-DL, xmax+DL, DL) # torso positions
    yticks = np.arange(ymin-DL, ymax+DL, DL)

    collision_cost = 1e9
    left_fail_cost = 1000000. if left_used else 100
    right_fail_cost = 1000000. if right_used else 100
    dist_cost = 1.

    base_costs = np.zeros((len(rars), xticks.size, yticks.size))
    coll_mask = get_collision_mask(xticks, yticks)
    base_costs += collision_cost * coll_mask[None,:,:]
    
    for (i, (xl, yl, zl), (xr, yr, zr)) in zip(xrange(len(rars)), xyz_l, xyz_r):
        zlind = intround(  (zl - ztorso - zticksir[0]) / DL  )
        zrind = intround(  (zr - ztorso - zticksir[0]) / DL  )
                
        base_costs[i] += (shift_and_place_image(invreachL[:,:,zlind], xl, yl, xticksir, yticksirL, xticks, yticks) <= 0) * left_fail_cost + dist_cost
        base_costs[i] += (shift_and_place_image(invreachR[:,:,zrind], xr, yr, xticksir, yticksirR, xticks, yticks) <= 0) * right_fail_cost + dist_cost
            
    xinds_base, yinds_base = get_feasible_path(base_costs).T
    return np.c_[xticks[xinds_base] - xtorso, yticks[yinds_base] - ytorso]


def get_base_positions_bimanual_resampled(rars, joints):
    n_orig = len(rars)
    
    rars_ds = rars[::10]
    n_ds = len(rars_ds)
    base_xys_ds = get_base_positions_bimanual(rars_ds, joints)
    
    base_xys = mu.interp2d(np.linspace(0,1,n_orig), np.linspace(0,1,n_ds), base_xys_ds)
    
    return base_xys