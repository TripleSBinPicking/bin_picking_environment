"""
Author:       Niels de Boer
Date:         10-10-2020
Description:  Planner can be used to plan movements using MoveIt!
"""
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import tf
from .util import rosparamOrDefault

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
        self.pose_reference_frame = rosparamOrDefault('~pose_reference_frame', 'base_link')
        self.move_group.set_pose_reference_frame(
            self.pose_reference_frame
        )
        
        # Setup tf
        self.tf = tf.TransformListener()

        # Create publisher to publish the planned trajectories
        self.publisher_display_trajectory = rospy.Publisher(
            rosparamOrDefault('~display_trajectory_publisher', '/move_group/display_planned_path'),
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=10
        )
    
    def displayPlan(self, plan):
        """
        Publish a planning on the self.publisher_display_trajectory topic

        plan -- The plan to publish
        """
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.publisher_display_trajectory.publish(display_trajectory)

    def planNamedTarget(self, target_name):
        """
        Create a planning to a named target.
        Named targets are defined in srdf and describe
        the joint positions of a move group
        
        target_name -- The name (as defined in srdf) of the target to move to

        returns -- tuple: planning succes (bool), plan
        """

        # Set the target
        self.move_group.set_named_target(target_name)
        
        # Create the planning
        plan = self.move_group.plan()

        self.displayPlan(plan)

        return len(plan.joint_trajectory.points) > 0, plan
    
    def executePlan(self, plan, wait=True):
        """
        Execute a plan
        
        plan -- Plan to execute (returned by move_group.plan())
        wait -- Only return this function if the movement is done, otherwise
                method returns before the movement is finished
        """
        return self.move_group.execute(plan, wait)

    def planAndExecuteNamedTarget(self, target_name, wait=True):
        """
        Plan to a named target and execute it directly

        target_name -- The name of the target (string)
        wait -- Only return this function if the movement is done, otherwise
                method returns before the movement is finished
        
        returns -- Was the execution successfull?
        """
        plan_success, plan = self.planNamedTarget(target_name)

        if plan_success:
            return self.executePlan(plan, wait=wait)
        else:
            return False
    
    def planInRefrenceFrame(self, poseStamped):
        """
        Plan to a pose that is not in the main refrence frame.
        Method tries to transform the pose into the local reference frame.

        poseStamped -- pose message with frame_id (geometry_msgs/PoseStamped)
        
        returns -- tuple: planning succes (bool), plan
        """
        if self.tf.frameExists(self.pose_reference_frame) and self.tf.frameExists(poseStamped.header.frame_id):
            # Transform the pose from the local frame to the planning reference frame
            pose_in_local_frame = self.tf.transformPose(self.pose_reference_frame, poseStamped)

            self.move_group.set_pose_target(pose_in_local_frame)
            
            plan = self.move_group.plan()

            self.displayPlan(plan)

            return len(plan.joint_trajectory.points) > 0, plan
        else:
            rospy.logwarn('Tried creating a planning to a frame that doesn\'t exists! Frames: %s and %s', (
                self.pose_reference_frame, poseStamped.header.frame_id
            ))
            return False, None

    def planAndExecuteInReferenceFrame(self, poseStamped, wait=True):
        """
        Plan and move to a pose that is not in the main reference frame.

        poseStamped -- pose message with frame_id (geometry_msgs/PoseStamped)
        wait -- Only return this function if the movement is done, otherwise
                method returns before the movement is finished
        
        return -- Was the movement successfull?
        """
        plan_success, plan = self.planInRefrenceFrame(poseStamped)

        if plan_success:
            return self.executePlan(plan, wait=wait)
        else:
            return False