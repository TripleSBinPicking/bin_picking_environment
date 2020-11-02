#!/usr/bin/env python
#
# Author:       Niels de Boer
# Date:         28-10-2020
# Description:  Move the robot between named target poses.
# Usage:        rosrun triple_s_util planner_script.py
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
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.move_group.set_pose_reference_frame('base_link')

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.printRobotInformation()

        self.startMoving()

    def printRobotInformation(self):
        """Print the information about the current state of the robot"""

        # Name of the reference frame for this robot:
        self.planning_frame = self.move_group.get_planning_frame()
        print "============ Planning frame: %s ============" % self.planning_frame

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        print "============ End effector link: %s ============" % self.eef_link

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print "============ Available Planning Groups:", self.robot.get_group_names(), "============"

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state: ============"
        print self.robot.get_current_state()

    def planNamedTarget(self, target_name):
        """ Create a planning to a named target.
            Named targets are define in srdf and describe
            the joint positions of a move group """
        # Set the target
        self.move_group.set_named_target(target_name)
        
        # Create the planning
        plan = self.move_group.plan()

        # Publish the planning (so rviz can visualize it)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        return plan

    def executePlan(self, plan, wait=True):
        """ Execute a plan """
        self.move_group.execute(plan, wait)

    def startMoving(self):
        plan = self.planNamedTarget('fold')

        rospy.sleep(2)

        self.executePlan(plan)
        plan = self.planNamedTarget('look_at_card')

        rospy.sleep(2)

        self.executePlan(plan)
        plan = self.planNamedTarget('camera_1')

        rospy.sleep(2)

        self.executePlan(plan)
        plan = self.planNamedTarget('camera_2')

        rospy.sleep(2)

        self.executePlan(plan)

        self.startMoving()

def main():
    try:
        planner = PlannerScript()
    except rospy.ROSInterruptException:
        return

if __name__ == '__main__':
    main()