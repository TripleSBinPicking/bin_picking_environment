#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class PlannerScript:
    def __init__(self):
    
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('planner_script', anonymous=True)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        self.move_group.set_pose_reference_frame('base_link')

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.printRobotInformation()

        self.moveToStartingState()
        print "============ Robot in starting state ============"

        self.goToCoords(1/2, 1/2, 1/2, 1/2)

    def printRobotInformation(self):
        """Print the information about the current state of the robot"""

        # Name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s" % self.planning_frame

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s" % self.eef_link

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state:"
        print self.robot.get_current_state()
        print "============"

    def moveToStartingState(self, wait=True):
        """Move the arm to a starting position

        Parameters
        ----------
        wait : bool, optional
            Wait for the action to be completed
        """

        joint_goal = self.move_group.get_current_joint_values()

        joint_goal[0] = pi/2
        joint_goal[1] = -pi/2
        joint_goal[2] = pi/2
        joint_goal[3] = -pi/2
        joint_goal[4] = pi/2
        joint_goal[5] = -pi/2

        self.move_group.go(joint_goal, wait=True)

        self.move_group.stop()

    def goToCoords(self, x, y, z, w):
        goal = geometry_msgs.msg.Pose()
        goal.orientation.w = w
        goal.position.x = x
        goal.position.y = y
        goal.position.z = z

        self.move_group.set_pose_target(goal)

        plan = self.move_group.go(wait=True)

        self.move_group.stop()

        self.move_group.clear_pose_targets()

        return plan



def main():
    try:
        planner = PlannerScript()
    except rospy.ROSInterruptException:
        return

if __name__ == '__main__':
    main()