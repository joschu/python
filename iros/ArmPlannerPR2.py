from IKPlannerFunctions import IKInterpolationPlanner
from brett2.PR2 import PR2, Arm, IKFail

class PlannerArm(Arm):
    """
    Planner class for the Arm.
    """
    def __init__ (self, pr2, rl):
        Arm.__init__(self, pr2, rl)
        self.planner = IKInterpolationPlanner(self, self.lr) 

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
        self.pr2.update_rave()
        trajectory = self.planner.goInDirection(d, dist, steps)
        
        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail


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
        self.pr2.update_rave()
        trajectory = self.planner.goInWorldDirection(d, dist, steps)
        
        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail
        

    def circleAroundRadius (self, d, rad, finAng, steps=10):
        """
        Moves the gripper in a circle.
        
        Direction of circle (either inner or outer)       -> d
        Radius of circle                                  -> rad
        Final angle covered by circle                     -> finAng
        Number of points of linear interpolation of angle -> steps
        """        
        self.pr2.update_rave()
        trajectory = self.planner.circleAroundRadius (d, rad, finAng)

        if trajectory: 
            self.follow_joint_trajectory (trajectory)
        else: raise IKFail
        
        

class PlannerPR2 (PR2):
    """
    Planner class for PR2 with planning arms.
    """    
    def __init__ (self):
        PR2.__init__(self)
        self.rarm = PlannerArm(self,'r')
        self.larm = PlannerArm(self, 'l')
        
    def gotoArmPosture (self, pos):
        self.larm.goto_posture(pos)
        self.rarm.goto_posture(pos)
        self.update_rave()

