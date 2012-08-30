import reachability as reach
import numpy as np
import h5py
import trajectory_library
import jds_utils.transformations as tf


reach.init(True)
tlj = reach.PR2.GetJoint("torso_lift_joint")

reach.PR2.SetDOFValues(tlj.GetLimits()[1], [tlj.GetDOFIndex()])
library = trajectory_library.TrajectoryLibrary("drivingtest.h5","read")
f = np.load("/home/joschu/bulletsim/data/knots/l_reachability.npz")

#states = np.zeros(100, trajectory_library.TrajectoryPoint)
#states["xyz_l"][:,1] += np.linspace(0,1,100)
#states["xyz_l"][:,2] += .9

states = np.array(library.root["segments"]["demo0.0"])
base_xys = reach.get_base_positions_bimanual_resampled(states, reach.PR2.GetDOFValues())


base_xyzs = np.c_[base_xys, np.zeros((len(base_xys),1))]

handles = []
handles.append(reach.ENV.drawlinestrip(points=base_xyzs,
                                       linewidth=3.0))                                       

handles.append(reach.ENV.drawlinestrip(points=states["xyz_r"],
                                       linewidth=3.0))                                       
handles.append(reach.ENV.drawlinestrip(points=states["xyz_l"],
                                       linewidth=3.0,colors=(0,0,1)))                                       


for i in xrange(len(base_xys)):
    base_tf = np.eye(4)
    base_tf[0:2,3] = base_xys[i]

    xlarm,ylarm,zlarm = states["xyz_l"][i]
    xrarm,yrarm,zrarm = states["xyz_r"][i]
    
    larm_tf = tf.quaternion_matrix(states["quat_l"][i])
    rarm_tf = tf.quaternion_matrix(states["quat_r"][i])
    larm_tf[:3,3] += states["xyz_l"][i]
    rarm_tf[:3,3] += states["xyz_r"][i]

    
    with reach.ENV:
        reach.PR2.SetTransform(base_tf)       
        
        
        dofvalues = reach.LEFT.FindIKSolution(larm_tf, 2+8)
        if dofvalues is not None:
            reach.PR2.SetDOFValues(dofvalues, reach.LEFT.GetArmIndices())
        else:
            print "left ik failed"
        
        dofvalues = reach.RIGHT.FindIKSolution(rarm_tf, 2+8)
        if dofvalues is not None:
            reach.PR2.SetDOFValues(dofvalues, reach.RIGHT.GetArmIndices())
        else:
            print "right ik failed"
        