#!/usr/bin/env python
"""
Author:       Niels de Boer
Date:         11-11-2020
Description:  FindObject starts a service which will return the best object location
              for a certain object.
"""
import sys
import rospy
import sensor_msgs.msg
import visualization_msgs.msg
import triple_s_util.srv
import message_filters
import copy

from triple_s_util.bin_picking.util import rosparamOrDefault
from triple_s_util.bin_picking.dope_collection import DopeCollection
from triple_s_util.bin_picking.forward_image import ForwardImage

class FindObject:
    def __init__(self):
        self.dopeCollection = DopeCollection()
        self.forwardImage = ForwardImage(
            input_camera_raw = rosparamOrDefault('~camera_raw', '/d435_sim/camera_raw'),
            input_camera_info = rosparamOrDefault('~camera_info', '/d435_sim/camera_info'),
            output_camera_raw = rosparamOrDefault('~dope_camera_raw', '/dope/camera_raw'),
            output_camera_info = rosparamOrDefault('~dope_camera_info', '/dope/camera_info')
        )

        # Setup service
        self.service = rospy.Service(
            rosparamOrDefault('~object_request_service', '/object_request'),
            triple_s_util.srv.ObjectRequest,
            self.onObjectRequest
        )

        rospy.loginfo('Done initializing find_object.py')

    def onObjectRequest(self, request):
        """
        Called when the object_request service is called.
        
        request -- The object that is requested (triple_s_util/ObjectRequest)
        """
        rospy.loginfo('Requesting object of type: %s' % request.object_name)

        self.forwardImage.forwardNext()
        self.dopeCollection.reset()
        
        # Wait until the data from DOPE is received
        while not self.dopeCollection.isComplete():
            rospy.sleep(0.1)

        # Fetch the result
        poses, markers, detections = self.dopeCollection.getResults()
        self.dopeCollection.reset()

        # Filter poses for this type
        requested_poses = self.filterPosesForObject(request.object_name, poses)

        # Create response instance
        result = triple_s_util.srv.ObjectRequestResponse()

        if len(requested_poses) == 0:
            rospy.loginfo('DOPE didn\'t detect any objects of type \"%s\"' % request.object_name)
            result.found_object = False
        else:
            best_pose = self.chooseBestPose(requested_poses)

            rospy.loginfo('Best pose for %s found at [%2.05f, %2.05f, %2.05f]:' % (
                request.object_name,
                best_pose.pose.position.x,
                best_pose.pose.position.y,
                best_pose.pose.position.z
                )
            )
            result.found_object = True
            result.object_pose = best_pose

        return result
    
    def filterPosesForObject(self, object_name, poses):
        """
        Find the poses that belong to a specific object

        object_name -- The object that we are trying to find poses for
        poses -- The list of poses in a tuple (object_name, pose)
        """
        return [pose for object_type, pose in poses if object_type == object_name]
    
    def chooseBestPose(self, pose_results):
        """ Find the best pose to move to (WIP)"""
        if len(pose_results) > 0:
            # TODO actually choose the best option
            return pose_results[0]
        else:
            return None
        
if __name__ == '__main__':
    rospy.init_node('find_object', anonymous=True)
    
    rospy.loginfo('Initializing find_object.py')

    FindObject()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exiting...')