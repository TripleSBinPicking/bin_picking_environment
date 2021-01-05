#!/usr/bin/env python
"""
Author:       Niels de Boer
Date:         05-01-2021
Description:  Set the frame id of marker messages
"""
import rospy
from visualization_msgs.msg import MarkerArray
from triple_s_util.bin_picking.util import rosparamOrDefault

publisher = None
topic = None
frame_id = None

def onMarkerReceived(marker):
    """
    Called when a marker is received. Adjust the frame id, and send the marker
    on the other topic

    ---

    marker -- markers (visualization_msgs/MarkerArray)
    """
    global publisher, topic, frame_id

    if publisher == None:
        rospy.logerr('Publisher not initialized!')
        return

    for i in range(0, len(marker.markers)):
        marker.markers[i].header.frame_id = frame_id
    
    publisher.publish(marker)

def main():
    """ Initialize the nodes """
    global publisher, topic, frame_id
    rospy.init_node('set_reference_frame', anonymous=True)
    rospy.loginfo('Starting set_marker_frame.py')

    topic = '/' + rosparamOrDefault('/dope/topic_publishing', 'dope') + '/markers'
    frame_id = rosparamOrDefault('/bin_picking/camera_link', 'camera_sim_link')
    rospy.Subscriber(topic, MarkerArray, onMarkerReceived)
    publisher = rospy.Publisher(topic + '_fixed', MarkerArray, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()

