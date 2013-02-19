import rospy
import numpy as np
import brett2.trajectories as tj
from brett2.PR2 import PR2
import trajectory_msgs.msg as tm

if __name__ == "__main__":
    rospy.init_node("trajectory_playback")
    p = PR2 ()

    traj_file_path = '/home/mallory/fuerte_workspace/sandbox/suturing/pr2_suturing/joint_states_listener/trajectories'
    traj_name = []
    ## Trajectory Options
    #traj_name.append('/full_running_suture/pt1')  #pierce both flaps and pull through
    #traj_name.append('/full_running_suture/pt2')  #tie right 2 loop knot
    #traj_name.append('/full_running_suture/pt3')  #tie left 1 loop knot
    #traj_name.append('/full_running_suture/pt4')  #tie right 1 loop knot
    #traj_name.append('/full_running_suture/pt5')  #tie left 1 loop knot
    #traj_name.append('/full_running_suture/pt6')  #first stitch of running suture
    #traj_name.append('/full_running_suture/pt7')  #second stitch of running suture
    #traj_name.append('/full_running_suture/pt8')  #third stitch of running suture
    #traj_name.append('/full_running_suture/pt9')  #final stitch of running suture
    #traj_name.append('/full_running_suture/pt10') #final knot pt1 
    #traj_name.append('/full_running_suture/pt11') #final knot pt2 --- thread was approx 11.5 feet long

    traj_name.append('/full_interrupted_suture/pt1')  #stitch 1, pierce both flaps and pull through
    traj_name.append('/full_interrupted_suture/pt2')  #tie right 2 loop knot
    traj_name.append('/full_interrupted_suture/pt3')  #tie left 1 loop knot
    traj_name.append('/full_interrupted_suture/pt4')  #tie right 1 loop knot
    traj_name.append('/full_interrupted_suture/pt5')  #tie left 1 loop knot
    traj_name.append('/full_interrupted_suture/pt6')  #cut tails off
    traj_name.append('/full_interrupted_suture/pt7')  #stitch 2, pierce both flaps and pull through
    traj_name.append('/full_interrupted_suture/pt8')  #tie right 2 loop knot
    traj_name.append('/full_interrupted_suture/pt9')  #tie left 1 loop knot --- weird smoothing at the end
    traj_name.append('/full_interrupted_suture/pt10') #tie right 1 loop knot
    traj_name.append('/full_interrupted_suture/pt11') #tie left 1 loop knot
    traj_name.append('/full_interrupted_suture/pt12') #cut tails off

    ## Load Trajectory
    for i in range(1):
        i=11
        larm_traj = np.load(traj_file_path + traj_name[i] + '_tj_larm.npy')
        rarm_traj = np.load(traj_file_path + traj_name[i] + '_tj_rarm.npy')
        lgrip_traj = np.load(traj_file_path + traj_name[i] + '_tj_lgrip.npy')
        rgrip_traj = np.load(traj_file_path + traj_name[i] + '_tj_rgrip.npy')
        
        print 'loaded', traj_name[i], 'trajectory'

        length = len(larm_traj)

        full_traj = np.zeros(length, 
                  dtype=[('r_arm', '7float64'), ('l_arm', '7float64'), ('r_gripper', 'float64'), ('l_gripper', 'float64')])

        full_traj['r_arm'] = rarm_traj
        full_traj['l_arm'] = larm_traj
        full_traj['r_gripper'] = rgrip_traj    
        full_traj['l_gripper'] = lgrip_traj
    
        raw_input("Confirm trajectory type, press ENTER to start playback ...")   

        tj.follow_body_traj(p, full_traj, r_arm = True, l_arm = True, r_gripper = True, l_gripper = True)

