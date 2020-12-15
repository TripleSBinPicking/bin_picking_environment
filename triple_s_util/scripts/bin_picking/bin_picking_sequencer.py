#!/usr/bin/env python
"""
Author:       Niels de Boer
Date:         11-11-2020
Description:  TBD
"""
import sys
import rospy
import triple_s_util.srv
import onrobot_rg2.srv
import geometry_msgs.msg
import moveit_msgs.msg
from triple_s_util.bin_picking.util import rosparamOrDefault
from triple_s_util.bin_picking.planner import Planner
from tf_conversions import transformations
import numpy as np
import math
from pyquaternion import Quaternion

class BinPickingSequencer():
    def __init__(self):
        self.planner = Planner()
        self.approach_distance = 0.2
        self.pick_up_config = rosparamOrDefault('/bin_picking/cylindrical_axis', {})
        self.z_limit = rosparamOrDefault('/bin_picking/rodrigues_z_limit', 0.8)

        object_request_service_name = rosparamOrDefault('/bin_picking/object_request_service', '/object_request')
        rospy.wait_for_service(object_request_service_name)
        self.requestObjectPose = rospy.ServiceProxy(object_request_service_name, triple_s_util.srv.ObjectRequest)
        self.controlGripper = rospy.ServiceProxy(rosparamOrDefault('/bin_picking/gripper_service', '/control_rg2'), onrobot_rg2.srv.ControlRG2)
        self.posePublisher = rospy.Publisher('/bin_picking/approach_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

        self.sequence()

    def sequence(self):
        object_to_request = 'tomatosauce'

        # Temporary: Move to start position
        rospy.loginfo('Moving to start position')
        self.planner.planAndExecuteNamedTarget('dope_to_rviz')
        self.controlGripper(110)

        # Request a pose of an object through the service
        rospy.loginfo('Requesting object position')
        request = self.requestObjectPose(object_to_request)
        rospy.loginfo('Got object position')

        if request.found_object:            
            approach_pose, pick_pose = self.determinePoses(request.object_pose, object_to_request)

            if not self.planner.planAndExecutePose(approach_pose):
                rospy.logwarn('Couldn\'t move into approach position to grab the object!')
            else:
                rospy.loginfo('Moved to approach pose')
                rospy.sleep(2)
                if not self.planner.planAndExecutePose(pick_pose):
                    rospy.logwarn('Couldn\'t move into position to grab the object!')
                else:
                    rospy.loginfo('Moved to object')
                    rospy.sleep(2)
                    self.controlGripper(0)
        else:
            rospy.logwarn('Couldn\'t find any objects of type \"%s\"' % object_to_request)


        rospy.sleep(10)

        self.planner.planAndExecuteNamedTarget('handoff')
        self.controlGripper(110)

        # Sleep for a bit
        rospy.sleep(5)

        # Repeat
        self.sequence()

    def determinePoses(self, object_pose, object_type):
        """
        Calculate the approach pose and the grasp pose for an object

        object_pose -- geometry_msgs/PoseStamped object pose
        object_type -- string, object name

        returns -- approach pose (geometry_msgs/PoseStamped), grasp pose (geometry_msgs/PoseStamped)
        """
        quaternion_object = Quaternion(
            object_pose.pose.orientation.w,
            object_pose.pose.orientation.x,
            object_pose.pose.orientation.y,
            object_pose.pose.orientation.z
        )

        # Get the approach unit vector and pose rotation
        approach, rotation = self.approachCalculator(quaternion_object, object_type)

        # Calculate the approach position:
        # approach * approach distance + object position
        approach_position = np.array(approach) * self.approach_distance + np.array([
            object_pose.pose.position.x,
            object_pose.pose.position.y,
            object_pose.pose.position.z
        ])
        
        # Create Pose message for approach
        approach_message = self.makePoseMessage(approach_position, rotation)
        
        # Adjust grasp orientation
        object_pose.pose.orientation = approach_message.orientation

        # Create PoseStamped message for debugging purposes
        pstmpd = geometry_msgs.msg.PoseStamped()
        pstmpd.header.frame_id = '/base_link'
        pstmpd.pose = approach_message
        self.posePublisher.publish(pstmpd)

        return approach_message, object_pose

    def approachCalculator(self, quaternion_object, object_type):
        """
        Calculate the approach unit vector (grasp diretion) and grasp orientation

        quaternion_object -- Quaternion of the rotation of the object
        object_type -- Name of the object

        return -- Unit vector (array), grasp orientation (Quaternion)
        """
        rotated_x = np.array(quaternion_object.rotate([1, 0, 0]))
        rotated_y = np.array(quaternion_object.rotate([0, 1, 0]))
        rotated_z = np.array(quaternion_object.rotate([0, 0, 1]))

        preffered_rotation, rod_vector, cylindrical_axis = self.getPrefferedRotation(object_type, rotated_x, rotated_y, rotated_z)

        if cylindrical_axis is not None and abs(preffered_rotation[2]) < self.z_limit:
            return self.getGraspPoseUsingRodrigues(preffered_rotation, rod_vector)
        else:
            return self.getGraspPoseUsingAxis(rotated_x, rotated_y, rotated_z)

    def getGraspPoseUsingAxis(self, rotated_x, rotated_y, rotated_z):
        """
        Get the grasping position using the axis of the grasping object.

        The axis of the object that is the highest is the approach axis for grasping.

        rotated_x -- X rotation vector
        rotated_y -- Y rotation vector
        rotated_z -- Z rotation vector

        return Unit vector, grasp orientation
        """
        rospy.loginfo('Determining grasping poses using axis rotation')

        array = np.zeros([3, 3])
        array[0] = rotated_x
        array[1] = rotated_y
        array[2] = rotated_z

        if abs(array[0][2]) < abs(rotated_y[2]):
            array[0] = rotated_y
            array[1] = rotated_z
            array[2] = rotated_x
        elif abs(array[0][2]) < abs(rotated_z[2]):
            array[0] = rotated_z
            array[1] = rotated_x
            array[2] = rotated_y

        pos = array[0]

        if pos[2] < 0:
            pos = -pos

        array = np.rot90(np.fliplr(array))
        quad = Quaternion(matrix=array)

        return pos, quad

    def getGraspPoseUsingRodrigues(self, preffered_rotation, rod_vector):
        """
        Get the grasping position using rodrigues rotation.

        Only suitable for cyclindrical objects. The highest point above the object is used
        as the grasping approach.

        preffered_rotation -- The axis to rotate around
        rod_vector -- Axis perpendicular to preffered_rotation

        returns -- Unit vector, grasp orientation
        """
        rospy.loginfo('Determining grasping poses using rodrigues rotation')
            
        rodr = self.rodriguesRotation(rod_vector, preffered_rotation)

        if preffered_rotation[2] < 0:
            preffered_rotation = -1 * preffered_rotation

        array = np.zeros([3, 3])
        array[0] = -rodr
        array[1] = np.cross(rodr, preffered_rotation)
        array[2] = preffered_rotation

        array = np.rot90(np.fliplr(array))
        quad = Quaternion(matrix=array)
        
        return rodr, quad

    def getPrefferedRotation(self, object_type, rotated_x, rotated_y, rotated_z):
        """
        Get the preffered rotation axis. This is determined by the config

        object_type -- name of the object
        rotated_x -- X rotation vector
        rotated_y -- Y rotation vector
        rotated_z -- Z rotation vector

        returns -- preffered rotation vector, perpendicular vector, cylindrical axis
        """
        if object_type in self.pick_up_config:
            cylindrical_axis = self.pick_up_config[object_type]
        else:
            cylindrical_axis = None

        preffered_rotation = rotated_x
        rod_vector = rotated_z

        if cylindrical_axis == 'y':
            preffered_rotation = rotated_y
            rod_vector = rotated_x
        elif cylindrical_axis == 'z':
            preffered_rotation = rotated_z
            rod_vector = rotated_y

        return preffered_rotation, rod_vector, cylindrical_axis

    def makePoseMessage(self, position, orientation):
        """
        Create a geometry_msgs/Pose message from coords and Quaternion

        position -- Array with the x, y, z coordinates ([x, y, z])
        orientation -- Quaternion object
        """
        message = geometry_msgs.msg.Pose()
        message.position.x = position[0]
        message.position.y = position[1]
        message.position.z = position[2]
        message.orientation.x = orientation[1]
        message.orientation.y = orientation[2]
        message.orientation.z = orientation[3]
        message.orientation.w = orientation[0]
        
        return message

    def rodriguesRotation(self, v, k):
        """
        Rotates vector v about unitvector k according to Rodrigues' rotation formula and returns highest point
        """
        temp = np.zeros(shape=(100,3))
        for i in range(1,100):
            temp[i] = v * math.cos(2*math.pi/i) + np.cross(k,v) * math.sin(2*math.pi/i) + k * np.dot(k,v) * (1-math.cos(2*math.pi/i))   
        comp = np.zeros(shape=(3))

        # Finds vector with highest z component
        for r in range(1,100):
            if temp[r-1][2] > comp[2]:
                comp = temp[r-1]

        return comp


    
if __name__ == '__main__':
    rospy.init_node('bin_picking_sequencer', anonymous=True)
    
    rospy.loginfo('Initializing bin_picking_sequencer.py')

    BinPickingSequencer()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exiting...')