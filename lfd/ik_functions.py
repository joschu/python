import rospy
from lfd import traj_ik_graph_search
import numpy as np
from jds_utils import conversions as juc
from brett2 import trajectories
import openravepy as rave
import IPython

def get_manipulator(pr2, lr):
    manip_name = {"l":"leftarm", "r":"rightarm"}[lr]
    manip = pr2.robot.GetManipulator(manip_name)
    return manip

# do ik using the graph search algorithm
def do_traj_ik_graph_search(pr2, lr, gripper_poses):
    manip = get_manipulator(pr2, lr)
    hmats = [juc.pose_to_hmat(pose) for pose in gripper_poses]

    def ikfunc(hmat):
        return traj_ik_graph_search.ik_for_link(hmat, manip, "%s_gripper_tool_frame"%lr, return_all_solns=True)
    
    pr2.update_rave()
    start_joints = pr2.robot.GetDOFValues(manip.GetArmJoints())

    robot = manip.GetRobot()
    env = robot.GetEnv()

    report = rave.CollisionReport()
    link_info = []

    def nodecost(joints):
        robot.SetDOFValues(joints, manip.GetArmJoints())
        return 100*env.CheckCollision(robot)
        
    paths, costs, timesteps = traj_ik_graph_search.traj_cart2joint(hmats, ikfunc, start_joints=start_joints, nodecost=nodecost)
    
    i_best = np.argmin(costs)
    print "lowest cost of initial trajs:", costs[i_best]
    best_path = paths[i_best]

    return best_path

# do ik using the openrave ik
def do_traj_ik_default(pr2, lr, gripper_poses):
    gripper_xyzs, gripper_quats = [], []
    for pose in gripper_poses:
        xyz, quat = juc.pose_to_trans_rot(pose)
        gripper_xyzs.append(xyz)
        gripper_quats.append(quat)
    manip = get_manipulator(pr2, lr)
    joint_positions, inds = trajectories.make_joint_traj(gripper_xyzs, gripper_quats, manip, "base_footprint", "%s_gripper_tool_frame"%lr, filter_options = 1+18)
    return joint_positions

# do ik by calling the plan_traj service
def do_traj_ik_opt(pr2, lr, gripper_poses):
    plan_traj_req = PlanTrajRequest()
    plan_traj_req.manip = "rightarm" if lr == 'r' else "leftarm"
    plan_traj_req.link = "%s_gripper_tool_frame"%lr
    plan_traj_req.task = "follow_cart"

    #set table bounds
    table_bounds = map(float, rospy.get_param("table_bounds").split())
    xmin, xmax, ymin, ymax, zmin, zmax = table_bounds
    plan_traj_req.xmlstring = \
    """
    <Environment>
        <KinBody name="%s">
            <Body type="static">
            <Geom type="box">
            <Translation> %f %f %f </Translation>
            <extents> %f %f %f </extents>
            </Geom>
            </Body>
        </KinBody>
    </Environment>
    """ \
    % ("table", 
      (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2,
      (xmax-xmin)/2, (ymax-ymin)/2, (zmax-zmin)/2)

    plan_traj_req.robot_joints = pr2.robot.GetDOFValues()

    pose_vals = []
    for pose in gripper_poses:
        xyz, quat = juc.pose_to_trans_rot(pose)
        pose_vals.append(np.concatenate((quat, xyz)))
    plan_traj_req.goal = np.array(pose_vals)[::5].flatten()

    plan_traj_service_name = "plan_traj"
    rospy.wait_for_service(plan_traj_service_name)
    plan_traj_service_proxy = rospy.ServiceProxy(plan_traj_service_name, PlanTraj)
    try:
        plan_traj_resp = plan_traj_service_proxy(plan_traj_req)
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
        return np.array([])

    joint_positions = np.array(plan_traj_resp.trajectory).reshape((-1, 7))
    return joint_positions

