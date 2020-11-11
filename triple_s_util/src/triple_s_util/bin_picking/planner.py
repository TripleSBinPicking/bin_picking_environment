"""
Author:       Niels de Boer
Date:         10-10-2020
Description:  Planner can be used to plan movements using MoveIt!
"""
import rospy
import moveit_commander
import moveit_msgs.msg
from .util import rosparamOrDefault

DEFAULT_REQUIRED_NAMED_POSES = ['look_at_card', 'bin_one', 'bin_two', 'pause', 'dropoff']

class Planner:
    """ Class that can plan movements """
    def __init__(self):
        # Init MoveIt!
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Setup the move group
        self.move_group = moveit_commander.MoveGroupCommander(
            rosparamOrDefault('~manipulator', 'manipulator')
        )

        # Set the reference frame
        self.move_group.set_pose_reference_frame(
            rosparamOrDefault('~pose_reference_frame', 'base_link')
        )

        # Create publisher to publish the planned trajectories
        self.publisher_display_trajectory = rospy.Publisher(
            rosparamOrDefault('~display_trajectory_publisher', '/move_group/display_planned_path'),
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10
        )

        if not self.checkNamedTargets():
            rospy.logwarn('Missing one or multiple named targets. bin_picking.py will not work as expected!')

    def checkNamedTargets(self):
        """ Validated that all the required poses are set """
        named_targets = self.move_group.get_named_targets()
        result = True

        for target_name in rosparamOrDefault('~pose_names', DEFAULT_REQUIRED_NAMED_POSES):
            if not target_name in named_targets:
                rospy.logwarn('Missing named target \"%s\"!' % target_name)
                result = False

        return result

    def planNamedTarget(self, target_name):
        """
        Create a planning to a named target.
        Named targets are defined in srdf and describe
        the joint positions of a move group
        
        target_name -- The name (as defined in srdf) of the target to move to
        """

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
        """
        Execute a plan
        
        plan -- Plan to execute (returned by move_group.plan())
        wait -- Only return this function if the movement is done, otherwise
                method returns before the movement is finished
        """
        self.move_group.execute(plan, wait)

    def planAndExecuteNamedTarget(self, target_name, wait=True):
        """ Plan to a named target and execute it directly """
        self.executePlan(self.planNamedTarget(target_name), wait=wait)