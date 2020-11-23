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
import geometry_msgs.msg
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
        
        if not rospy.has_param('/dope/class_ids'):
            rospy.logerr('FindObject: /dope/class_ids is not loaded on the parameter server!')
            sys.exit(1)

        self.classIds = rospy.get_param('/dope/class_ids')

        rospy.loginfo('Done initializing find_object.py')

    def onObjectRequest(self, request):
        """
        Called when the object_request service is called.
        
        request -- The object that is requested (triple_s_util/ObjectRequest)
        """
        rospy.loginfo('Requesting object of type: %s' % request.object_name)

        # Create response instance
        result = triple_s_util.srv.ObjectRequestResponse()

        # Get the class id for this object name
        class_id = self.getClassId(request.object_name)

        if class_id < 0:
            rospy.logwarn('Object not detectable because it is not registered in DOPE!')
            result.found_object = False
            return result

        self.forwardImage.forwardNext()
        self.dopeCollection.reset()
        
        # Wait until the data from DOPE is received
        while not self.dopeCollection.isComplete():
            rospy.sleep(0.1)

        # Fetch the result
        _, _, detections = self.dopeCollection.getResults()

        detections = self.filterDetectionsForClassId(class_id, detections.detections)
        
        if len(detections) == 0:
            rospy.loginfo('DOPE didn\'t detect any objects of type \"%s\"' % request.object_name)
            result.found_object = False
        else:
            best_detection = self.chooseBestDetection(detections)

            best_pose = geometry_msgs.msg.PoseStamped()
            best_pose.header = best_detection.header
            best_pose.pose = best_detection.results[0].pose.pose

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
    
    def filterDetectionsForClassId(self, class_id, detections):
        """
        Filter the 
        class_id -- Id of class to find
        detections -- vision_msgs/Detection3D[]
        """
        return [detection for detection in detections if detection.results[0].id == class_id]

    def chooseBestDetection(self, detections):
        """ Find the best pose to move to (WIP)"""
        if len(detections) > 0:
            # TODO actually choose the best option
            return detections[0]
        else:
            return None

    def getClassId(self, class_name):
        """
        Convert the class name into the class id of the object

        class_name -- the name of the object

        return -- the id of the object
        """
        if class_name in self.classIds:
            return self.classIds[class_name]
        else:
            return -1
        
        
if __name__ == '__main__':
    rospy.init_node('find_object', anonymous=True)
    
    rospy.loginfo('Initializing find_object.py')

    FindObject()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exiting...')