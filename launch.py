import rospy
from ArmPlannerPR2 import PlannerPR2

if __name__ == "__main__":
    rospy.init_node("Test_Planning")
    p = PlannerPR2 ()
