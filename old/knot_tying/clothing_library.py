import trajectory_library as tl, numpy as np

ClothTrajectoryPoint = np.dtype([("xyz_l",float,3),
                                ("quat_l",float,4),
                                ("grip_l",float),
                                ("grab_l",int),                            
                                ("xyz_r",float,3),
                                ("quat_r",float,4),
                                ("grip_r",float),
                                ("grab_r",int),
                                ("cloth",object)]) # either contour or none. maybe use a richer repr later
