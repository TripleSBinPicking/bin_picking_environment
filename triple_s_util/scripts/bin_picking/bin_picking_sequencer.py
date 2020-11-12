#!/usr/bin/env python
"""
Author:       Niels de Boer
Date:         11-11-2020
Description:  TBD
"""
import sys
import rospy
import triple_s_util.srv
from triple_s_util.bin_picking.util import rosparamOrDefault
from triple_s_util.bin_picking.planner import Planner

class BinPickingSequencer():
    def __init__(self):
        self.planner = Planner()

        object_request_service_name = rosparamOrDefault('~object_request_service', '/object_request')
        rospy.wait_for_service(object_request_service_name)
        self.requestObjectPose = rospy.ServiceProxy(object_request_service_name, triple_s_util.srv.ObjectRequest)
        self.sequence()

    def sequence(self):
        # Request a pose of an object through the service
        request = self.requestObjectPose('tomatosauce')

        # Set the reference frame of the pose to the camera
        request.object_pose.header.frame_id = 'camera_sim_link'

        if not self.planner.planAndExecuteInReferenceFrame(request.object_pose):
            rospy.logwarn('Couldn\'t move into position to grab the object!')

        # Sleep for a bit
        rospy.sleep(10)

        # Repeat
        self.sequence()

    
if __name__ == '__main__':
    rospy.init_node('bin_picking_sequencer', anonymous=True)
    
    rospy.loginfo('Initializing bin_picking_sequencer.py')

    BinPickingSequencer()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exiting...')