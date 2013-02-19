from brett2.PR2 import Arm
import kinematics.kinematics_utils as ku

import rospy

import numpy as np
from numpy.linalg import norm


class IKInterpolationPlanner(object):
    """
    Class which plans based on OpenRAVE's IK solver
    """    
    def __init__(self, _arm, _rl, _filter_options=0):
        self.rl = _rl
        #If useful
        self.rl_long = {'l':'left', 'r':'right'}[self.rl]
        self.arm = _arm
        self.filter_options = _filter_options
    
    
    def setFilterOptions (self, filter_options):
        """
        Filter options indicate basic options while planning:
    
        IKFO_CheckEnvCollisions = 1
        IKFO_IgnoreSelfCollisions = 2
        IKFO_IgnoreJointLimits = 4
        IKFO_IgnoreCustomFilters = 8
        IKFO_IgnoreEndEffectorCollisions = 16
        """
        self.filter_options = filter_options
        
        
   
    def plan(self, transforms):
        """        
        Basic plan function which just solves the IK for each transform given.
    
        List of transforms along path -> transforms
        """            
        if len(transforms) < 1:
            rospy.ERROR('Not enough transforms!')
        
        if len(transforms) == 1:
            firstTransform = self.arm.manip.GetEndEffectorTransform()
            transforms = [firstTransform, transforms[0]]
            
        trajectory = []
        
        for transform in transforms:
            joints = self.arm.manip.FindIKSolution(transform, self.filter_options)
            
            if joints is None:
                rospy.logwarn('IK Failed on ' + self.rl_long + 'arm.') 
                return joints
            
            trajectory.append(joints)
        
        return trajectory
    
    
    #Assuming all the Joint DOFs are numpy arrays
    def smoothPlan(self, transforms):
        """
        Smooth plan function which solves for all IK solutions for each transform given.
        It then chooses the joint DOF values which are closest to current DOF values.
        
        List of transforms along path -> transforms
        """        
        if len(transforms) < 1:
            rospy.ERROR('Not enough transforms!')
        
        if len(transforms) == 1:
            initTransform = self.arm.manip.GetEndEffectorTransform()
            transforms = [initTransform, transforms[0]]
            
        trajectory = []
        
        #armIndices = self.arm.manip.GetJointIndices()
        #currJoints = self.pr2.robot.GetDOFValues(armIndices)
        currJoints = self.arm.get_joint_positions()
        
        for transform in transforms:
            allJoints = self.arm.manip.FindIKSolutions(transform, self.filter_options)

            if not allJoints.size:
                rospy.logwarn('IK Failed on ' + self.rl_long + 'arm.')
                return None
            
            allJoints = [ku.closer_joint_angles(joints, currJoints) for joints in allJoints]
            normDifferences = [norm(currJoints-joints,2) for joints in allJoints]
            minIndex = normDifferences.index(min(normDifferences))
            
            trajectory.append(allJoints[minIndex])
        
        return trajectory
    
    
    #Not sure about scale here. Treating everything as meters. 
    #I don't think I need to worry about scale here anyway
    def goInDirection (self, d, dist, steps=10):
        """
        Moves the tool tip in the specified direction in the gripper frame.
         
        Direction of movement                    -> d
            f -> forward (along the tip of the gripper)
            b -> backward
            u -> up
            d -> down
            l -> left
            r -> right
        Distance traveled by tool tip            -> dist
        Number of points of linear interpolation -> steps
        """        
        initTransform = self.arm.manip.GetEndEffectorTransform()
        initOrigin = initTransform[0:3,3]
        
        if    d == 'f':
            dirVec = initTransform[0:3,2]
        elif  d == 'b': 
            dirVec = -1*initTransform[0:3,2]
        elif  d == 'u':
            dirVec = initTransform[0:3,1]
        elif  d == 'd':
            dirVec = -1*initTransform[0:3,1]
        elif  d == 'l':
            dirVec = initTransform[0:3,0]
        elif  d == 'r':
            dirVec = -1*initTransform[0:3,0]
        else:
            rospy.ERROR("Invalid direction: " + d)
        
        endOffset = dirVec*float(dist)
        
        transforms = []
        
        for step in range(steps+1):
            currVec = initOrigin + float(step)/steps*endOffset
            
            newTfm = initTransform.copy()
            newTfm[0:3,3] = np.unwrap(currVec)
            
            transforms.append(newTfm)
            
        return self.smoothPlan(transforms)

        
    
    #Not sure about scale here. Treating everything as meters. 
    #I don't think I need to worry about scale here anyway
    def goInWorldDirection (self, d, dist, steps=10):
        """
        Moves the tool tip in the specified direction in the base_link frame. 
        
        Direction of movement                    -> d
            f -> forward
            b -> backward
            u -> up
            d -> down
            l -> left
            r -> right
        Distance traveled by tool tip            -> dist
        Number of points of linear interpolation -> steps
        """
        initTransform = self.arm.manip.GetEndEffectorTransform()
        initOrigin = initTransform[0:3,3]
        baseTransform = self.arm.pr2.robot.GetLink('base_link').GetTransform()
        
        if    d == 'u':
            dirVec = baseTransform[0:3,2]
        elif  d == 'd': 
            dirVec = -1*baseTransform[0:3,2]
        elif  d == 'l':
            dirVec = baseTransform[0:3,1]
        elif  d == 'r':
            dirVec = -1*baseTransform[0:3,1]
        elif  d == 'f':
            dirVec = baseTransform[0:3,0]
        elif  d == 'b':
            dirVec = -1*baseTransform[0:3,0]
        else:
            rospy.ERROR("Invalid direction: " + d)
        
        endOffset = dirVec*float(dist)
        
        transforms = []
        
        for step in range(steps+1):
            currVec = initOrigin + endOffset*(float(step)/steps)
            
            newTfm = initTransform.copy()
            newTfm[0:3,3] = np.unwrap(currVec)
            
            transforms.append(newTfm)
            
        return self.smoothPlan(transforms)
  
        
    def circleAroundRadius (self, d, rad, finAng, steps=10):
        """
        Moves the gripper in a circle.
        
        Direction of circle (either inner or outer)       -> d
        Radius of circle                                  -> rad
        Final angle covered by circle                     -> finAng
        Number of points of linear interpolation of angle -> steps
        """
        WorldFromEETfm = self.arm.manip.GetEndEffectorTransform()
        
        initTfm = np.eye(4)
        initOrigin = d*rad*np.array([0,1,0])
        initTfm[0:3,3] = np.unwrap(initOrigin)
        transforms = []
        
        for step in range(steps+1):
            ang = float(finAng)*step/steps
            
            rotMat = np.eye(4)
            rotMat[0,0] = np.cos(d*ang)
            rotMat[0,1] = -np.sin(d*ang)
            rotMat[1,0] = np.sin(d*ang)
            rotMat[1,1] = np.cos(d*ang)
            rotMat[0:3,3] = np.unwrap(-1*initOrigin)
            print rotMat
            
            transforms.append(WorldFromEETfm.dot(rotMat.dot(initTfm)))
            
        return self.smoothPlan(transforms)
