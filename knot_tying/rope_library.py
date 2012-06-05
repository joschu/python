import trajectory_library as tl, numpy as np

N_CTRL_PTS = 100

RopeTrajectoryPoint = np.dtype([("xyz_l",float,3),
                                ("quat_l",float,4),
                                ("grip_l",float),
                                ("grab_l",int),                            
                                ("xyz_r",float,3),
                                ("quat_r",float,4),
                                ("grip_r",float),
                                ("grab_r",int),
                                ("rope",float,(N_CTRL_PTS,3))])


def calc_rope_dists(rope_k3, others_nk3):

    N = len(others_nk3)
    K = len(rope_k3)

    dists_n = np.empty(N)    

    hrope_k4 = np.c_[rope_k3, np.ones((K,1))]

    for (n,other_k3) in enumerate(others_nk3):
        hother_k4 = np.c_[other_k3, np.ones((K,1))]
        dists_n[n] = tl.affine_residual(hrope_k4, hother_k4)

    return dists_n


def transform_rope_traj(oldtraj, F):
    newtraj = oldtraj.copy()
    newtraj["xyz_l"], newtraj["quat_l"] = tl.transform_poses(oldtraj["xyz_l"],oldtraj["quat_l"],F)
    newtraj["xyz_r"], newtraj["quat_r"] = tl.transform_poses(oldtraj["xyz_r"],oldtraj["quat_r"],F)        

    old_rope_pts = oldtraj["rope"].reshape(-1,3)
    new_rope_pts = tl.transform_points(old_rope_pts, F)
    newtraj["rope"] = new_rope_pts.reshape(oldtraj["rope"].shape)
    return newtraj

class RopeTrajectoryLibrary(tl.TrajectoryLibrary):

    def lookup_closest(self, rope):
        "return the name of the closest trajectory segment"
        assert rope.shape[1] == 3
        
        initial_ropes = [seg["rope"][0] for seg in self.root["segments"].values()]
        
        dists = calc_rope_dists(rope, initial_ropes)
        print "dists:", dists
        i_min = dists.argmin()        
        return self.root["segments"].keys()[i_min]


    def get_closest_and_warp(self, rope_k3):
        demo_name = self.lookup_closest(rope_k3)
        traj_segment = np.array(self.root["segments"][demo_name])

        new_rope = traj_segment["rope"][0]
        if np.dot(new_rope[0] - new_rope[-1], rope_k3[0] - rope_k3[-1]) < 0:
            rope_k3 = rope_k3[::-1]

        F = tl.fit_transform(traj_segment["rope"][0], rope_k3)
        newsegment = transform_rope_traj(traj_segment, F)

        return traj_segment,newsegment


                                  
                                
