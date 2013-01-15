import trajoptpy
import openravepy as rave
import numpy as np
import json
import atexit # avoid segfault  at exit
import trajoptpy.math_utils as mu

def joints2xyzquat(manip, link_name, joint_vals):
    robot, T = manip.GetRobot(), None
    with robot:
        robot.SetDOFValues(joint_vals, manip.GetArmIndices())
        T = robot.GetLink(link_name).GetTransform()
    quat_xyz = rave.poseFromMatrix(T)
    return quat_xyz[4:7], quat_xyz[:4] # xyz, quat

def move_arm_straight_request(manip, n_steps, link_name, joints_start, joints_end, xyz_start, quat_start, xyz_end, quat_end):
    '''
    WARNING: start/end joints, xyz, and quats must all agree, otherwise the optimization problem will be infeasible!
    '''
    init_joints = mu.linspace2d(joints_start, joints_end, n_steps)
    request = {
        'basic_info': {
          'n_steps': n_steps,
          'manip': manip.GetName(),
          'start_fixed': False, # take care of this with the init_joints constraint instead
        },
        'costs': [
            { 'type': 'joint_vel', 'params': { 'coeffs': [1] } },
            { 'type': 'collision', 'params': { 'coeffs': [1], 'dist_pen': [0.025] } },
        ],
        'constraints': [
            {
                'type' : 'joint',
                'name' : 'init_joints',
                'params' : { 'vals': list(joints_start), 'timestep': 0 },
            },
            {
                'type' : 'joint',
                'name' : 'final_joints',
                'params' : { 'vals': list(joints_end) },
            },
        ],
        'init_info': {
            'type': 'given_traj',
            'data': [j.tolist() for j in init_joints],
        },
    }

    # add costs for deviation from a straight line in cartesian space
    lin_xyzs = mu.linspace2d(xyz_start, xyz_end, n_steps)
    lin_quats = [rave.quatSlerp(quat_start, quat_end, t) for t in np.linspace(0, 1, n_steps)]
    for i in range(1, n_steps-1):
        request['costs'].append({
            'type': 'pose',
            'params': {
                'xyz': list(lin_xyzs[i]),
                'wxyz': list(lin_quats[i]),
                'pos_coeffs': [1, 1, 1],
                'rot_coeffs': [1, 1, 1],
                'link': link_name,
                'timestep': i,
            }
        })

    return request

def move_arm_straight(env, manip, n_steps, link_name, joints_start, joints_end, xyz_start=None, quat_start=None, xyz_end=None, quat_end=None, interactive=False):
    '''
    Finds a trajectory (of length n_steps) such that the specified link travels
    as close as possible to a straight line in cartesian space, constrained to
    the specified starting and ending joint values.

    xyz_start, quat_start, xyz_end, quat_end are the starting and ending cartesian
    poses that correspond to joints_start and joints_end. Setting them to None
    will compute them here.
    WARNING: start/end joints, xyz, and quats must all agree, otherwise the optimization problem will be infeasible!

    Returns: trajectory (numpy.ndarray, n_steps x dof)
    '''
    if xyz_start is None or quat_start is None:
        xyz_start, quat_start = joints2xyzquat(manip, link_name, joints_start)
    if xyz_end is None or quat_end is None:
        xyz_end, quat_end = joints2xyzquat(manip, link_name, joints_end)
    request = move_arm_straight_request(manip, n_steps, link_name, joints_start, joints_end, xyz_start, quat_start, xyz_end, quat_end)
    trajoptpy.SetInteractive(interactive)
    prob = trajoptpy.ConstructProblem(json.dumps(request), env)
    result = trajoptpy.OptimizeProblem(prob)
    return result.GetTraj()


def move_arm_cart_request(manip, link_name, xyzs, quats, init_soln):
    n_steps = len(xyzs)
    request = {
        'basic_info': {
          'n_steps': n_steps,
          'manip': manip.GetName(),
          'start_fixed': False,
        },
        'costs': [
            { 'type': 'joint_vel', 'params': { 'coeffs': [1] } },
            { 'type': 'collision', 'params': { 'coeffs': [1], 'dist_pen': [0.025] } },
        ],
        'constraints': [],
        'init_info': {
            'type': 'given_traj',
            'data': [j.tolist() for j in init_soln],
        },
    }
    for i in range(n_steps):
        request['constraints'].append({
            'type': 'pose',
            'name': 'pose_%d' % i,
            'params': {
                'xyz': list(xyzs[i]),
                'wxyz': list(quats[i]),
                'link': link_name,
                'timestep': i,
            }
        })
    return request

def move_arm_cart(env, manip, link_name, xyzs, quats, init_soln, interactive=False):
    assert len(xyzs) == len(quats) == len(init_soln)
    request = move_arm_cart_request(manip, link_name, xyzs, quats, init_soln)
    trajoptpy.SetInteractive(interactive)
    prob = trajoptpy.ConstructProblem(json.dumps(request), env)
    result = trajoptpy.OptimizeProblem(prob)
    return result.GetTraj()


def play_traj(traj, env, manip):
    robot = manip.GetRobot()
    with env:
        for pt in traj:
            robot.SetDOFValues(pt, manip.GetArmIndices())
            env.UpdatePublishedBodies()
            raw_input('.')

def main():
        
    ### Parameters ###
    ENV_FILE = "../../trajopt/data/pr2_table.env.xml"
    MANIP_NAME = "rightarm"
    N_STEPS = 10
    LINK_NAME = "r_gripper_tool_frame"
    INTERACTIVE = True
    #joints_start_end = np.array([
    #    [-0.95, -0.38918253, -2.43888696, -1.23400121, -0.87433154, -0.97616443, -2.10997203],
    #    [-0.4, -0.4087081, -3.77121706, -1.2273375, 0.69885101, -0.8992004, 3.13313843]
    #])

    #joints_start_end = np.array([[0.34066373,   -0.49439586,   -3.3   ,       -1.31059503 ,  -1.28229698,   -0.15682819, -116.19626995],
    #    [   0.5162424 ,   -0.42037121 ,  -3.7     ,     -1.30277208  , 1.31120586,   -0.16411924 ,-118.57637204]])

    #joints_start_end = np.array([[  -1.83204054  , -0.33201855 ,  -1.01105089 ,  -1.43693186  , -1.099908,   -2.00040616, -116.17133393],
   #[  -0.38176851  ,  0.17886005  , -1.4    ,      -1.89752658 ,  -1.93285873,   -1.60546868, -114.70809047]])

    joints_start_end = np.array([[0.33487707,   -0.50480484 ,  -3.3    ,      -1.33546928  , -1.37194549 ,  -0.14645853 ,-116.11672039],  [  4.71340480e-01 , -4.56593341e-01 , -3.60000000e+00 , -1.33176173e+00,
   1.21859723e+00 , -9.98780266e-02,  -1.18561732e+02]])


    ##################
    joints_start_end[:,2] = np.unwrap(joints_start_end[:,2])
    joints_start_end[:,4] = np.unwrap(joints_start_end[:,4])
    joints_start_end[:,6] = np.unwrap(joints_start_end[:,6])
    joints_start = joints_start_end[0,:]
    joints_end = joints_start_end[1,:]
    
    
    ### Env setup ####
    env = rave.RaveGetEnvironment(1)
    if env is None:
        env = rave.Environment()
        env.StopSimulation()
        atexit.register(rave.RaveDestroy)
        env.Load(ENV_FILE)
    robot = env.GetRobots()[0]
    manip = robot.GetManipulator(MANIP_NAME)
    robot.SetDOFValues(joints_start, manip.GetArmIndices())
    ##################

    result_traj = move_arm_straight(env, manip, N_STEPS, LINK_NAME, joints_start, joints_end, interactive=INTERACTIVE)

    print 'Showing original straight-line trajectory'
    env.SetViewer('qtcoin')
    env.UpdatePublishedBodies()
    import time
    time.sleep(2)
    play_traj(mu.linspace2d(joints_start, joints_end, N_STEPS), env, manip)
    raw_input('press enter to continue')
    print 'Showing optimized trajectory'
    play_traj(result_traj, env, manip)
    raw_input('press enter to continue')

if __name__ == "__main__":
    main()
