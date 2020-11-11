#!/usr/bin/env python
#
# Author:       Niels de Boer
# Date:         10-10-2020
# Description:  TBD
# Usage:        TBD
import sys
import rospy
import sensor_msgs.msg
import visualization_msgs.msg
import triple_s_util.srv
import message_filters
import copy

from triple_s_util.bin_picking.util import rosparamOrDefault
from triple_s_util.bin_picking.dope_collection import DopeCollection
from triple_s_util.bin_picking.planner import Planner
from triple_s_util.bin_picking.forward_image import ForwardImage

class BinPicking:
    def __init__(self):
        # self.planner = Planner()
        self.dopeCollection = DopeCollection()
        self.forwardImage = ForwardImage(
            input_camera_raw = rosparamOrDefault('~camera_raw', '/d435_sim/camera_raw'),
            input_camera_info = rosparamOrDefault('~camera_info', '/d435_sim/camera_info'),
            output_camera_raw = rosparamOrDefault('~dope_camera_raw', '/dope/camera_raw'),
            output_camera_info = rosparamOrDefault('~dope_camera_info', '/dope/camera_info')
        )

        self.publishers = {}
        self.subscribers = {}

        # Setup services
        self.services = {}
        self.services['object_request'] = rospy.Service(
            rosparamOrDefault('~object_request_service', '/object_request'),
            triple_s_util.srv.ObjectRequest,
            self.onObjectRequest
        )

        rospy.loginfo('Done initializing bin_picking.py')

    def onObjectRequest(self, request):
        """ Called when the object_request service is called. """
        rospy.loginfo('Requesting object of type: %s' % request.object_name)

        self.forwardImage.forwardNext()
        self.dopeCollection.reset()
        
        # Wait until the data from DOPE is received
        while not self.dopeCollection.is_complete:
            rospy.sleep(0.1)

        # Fetch the result
        poses, markers = self.dopeCollection.getResults()
        self.dopeCollection.reset()

        # Filter poses for this type
        requested_poses = [(object_type, pose) for object_type, pose in poses if object_type == request.object_name]

        if len(requested_poses) == 0:
            rospy.loginfo('DOPE didn\'t detect any objects of type \"%s\"' % request.object_name)
            return False
        else:
            rospy.loginfo('Found poses:')
            rospy.loginfo(requested_poses)
            return True
        
def main():
    rospy.init_node('bin_picking', anonymous=True)
    
    rospy.loginfo('Initializing bin_picking.py')

    BinPicking()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exiting...')

if __name__ == '__main__':
    main()