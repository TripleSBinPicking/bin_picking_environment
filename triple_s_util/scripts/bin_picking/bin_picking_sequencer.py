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
        self.pick_up_config = rospy.get_param('/dope/pick_up_config')

        object_request_service_name = rosparamOrDefault('~object_request_service', '/object_request')
        rospy.wait_for_service(object_request_service_name)
        self.requestObjectPose = rospy.ServiceProxy(object_request_service_name, triple_s_util.srv.ObjectRequest)
        self.controlGripper = rospy.ServiceProxy(rosparamOrDefault('/dope/gripper_service', '/control_rg2'), onrobot_rg2.srv.ControlRG2)
        self.posePublisher = rospy.Publisher('/tmp_pose', geometry_msgs.msg.PoseStamped, queue_size=10)

        self.sequence()

    def sequence(self):
        object_to_request = 'BeerOpener'

        # Temporary: Move to start position
        rospy.loginfo('Moving to start position')
        self.planner.planAndExecuteNamedTarget('look_at_bin')
        self.controlGripper(110)

        # Request a pose of an object through the service
        rospy.loginfo('Requesting object position')
        request = self.requestObjectPose(object_to_request)
        rospy.loginfo('Got object position')

        if request.found_object:
            # Set the reference frame of the pose to the camera
            #request.object_pose.header.frame_id = rosparamOrDefault('~camera_link', 'camera_sim_link')
            
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
        print "Object Pose: ", object_pose

        quaternion_object = Quaternion(
            object_pose.pose.orientation.w,
            object_pose.pose.orientation.x,
            object_pose.pose.orientation.y,
            object_pose.pose.orientation.z
        )

        print "Object Quat: ", quaternion_object

        approach, rotation = self.approachCalculator(quaternion_object, object_type)

        print "Approach: ", approach
        print type(approach)

        approach_position = np.array(approach) * self.approach_distance + np.array([
            object_pose.pose.position.x,
            object_pose.pose.position.y,
            object_pose.pose.position.z
        ])

        print "Approach position: ", approach_position


        #for i in range(0, 25):
        #    rotation_rotated = rotation.rotate(Quaternion(axis=[0, 1, 0], angle=2*np.pi* (i/25)))



        approach_message = self.makePoseMessage(approach_position, rotation)
        object_pose.pose.orientation = approach_message.orientation
        pstmpd = geometry_msgs.msg.PoseStamped()

        pstmpd.header.frame_id = '/base_link'
        pstmpd.pose = approach_message
        self.posePublisher.publish(pstmpd)

        return approach_message, object_pose

    def approachCalculator(self, quaternion_object, object_type):
        rotated_x = np.array(quaternion_object.rotate([1, 0, 0]))
        rotated_y = np.array(quaternion_object.rotate([0, 1, 0]))
        rotated_z = np.array(quaternion_object.rotate([0, 0, 1]))

        if object_type in self.pick_up_config:
            obj_config = self.pick_up_config[object_type]
        else:
            obj_config = {
                'preffered_axis': 'z',
                'is_cylindrical': True
            }

        preffered_rotation = rotated_x
        rod_vector = rotated_z

        if obj_config['preffered_axis'] == 'y':
            preffered_rotation = rotated_y
            rod_vector = rotated_x
        elif obj_config['preffered_axis'] == 'z':
            preffered_rotation = rotated_z
            rod_vector = rotated_y

        

        if obj_config['is_cylindrical'] and abs(preffered_rotation[2]) < 1:
            print 'rodriguesRotation rotation'
            
            rodr = self.rodriguesRotation(rod_vector, preffered_rotation)

            if preffered_rotation[2] < 0:
                preffered_rotation = -1 * preffered_rotation

            array = np.zeros([3, 3])
            array[0] = -rodr
            array[2] = preffered_rotation
            array[1] = np.cross(rodr, preffered_rotation)

            array = np.rot90(np.fliplr(array))
            quad = Quaternion(matrix=array)
            
            return rodr, quad
        else:
            print 'Axis grab'

            # If the z component of the preffered axis is negative
            # the vector is inversed. So the object is picked up from the
            # other side
            if preffered_rotation[2] < 0:
                preffered_rotation = -1 * preffered_rotation
            
            return preffered_rotation, quaternion_object

    def makePoseMessage(self, position, orientation):
        message = geometry_msgs.msg.Pose()
        message.position.x = position[0]
        message.position.y = position[1]
        message.position.z = position[2]
        message.orientation.x = orientation[1]
        message.orientation.y = orientation[2]
        message.orientation.z = orientation[3]
        message.orientation.w = orientation[0]
        
        print message
        
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