import reachability as reach
import numpy as np
f = np.load("/home/joschu/bulletsim/data/knots/l_reachability.npz")

z = .7
y = 0 #left to right
xs = np.linspace(0,1,50)
xyzs = np.array([(x,y,z) for x in xs])

reach.init(True)
base_xys = reach.get_base_positions(xyzs)
base_xyzs = np.c_[base_xys, np.zeros((len(base_xys),1))]

handles = []
handles.append(reach.ENV.drawlinestrip(points=xyzs,
                                       linewidth=3.0))                                       

handles.append(reach.ENV.drawlinestrip(points=base_xyzs,
                                       linewidth=3.0))                                       


for i in xrange(len(base_xys)):
    base_tf = np.eye(4)
    base_tf[0:2,3] = base_xys[i]

    xarm, yarm, zarm = xyzs[i]
    arm_tf = np.array([[-1, 0,  0,  xarm],
                       [0, 0,  1,  yarm],
                       [0, 1,  0,  zarm],
                       [0, 0,  0,  1]])

    
    with reach.ENV:
        reach.PR2.SetTransform(base_tf)       
        dofvalues = reach.LEFT.FindIKSolution(arm_tf, 2+8)
        reach.PR2.SetDOFValues(dofvalues, reach.LEFT.GetArmIndices())
        
        