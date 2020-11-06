#!/usr/bin/env python
#
# Author:       Niels de Boer
# Date:         05-11-2020
# Description:  Set the frame_ids of a rosbag that contains messages of /dope,
#               also removes the rosout and webcam topics.
#               Only use on the commandline not in a node.
import rosbag
import argparse
from visualization_msgs.msg import MarkerArray

# Parse arguments
parser = argparse.ArgumentParser(description='Convert the frame ids of rosbag files')
parser.add_argument('--input', '-i', help='Input bagfile', default='input.bag')
parser.add_argument('--output', '-o', help='Output bagfile', default='output.bag')
parser.add_argument('--frame', '-f', help='The frame id to use', default='/camera_sim_link')
options = parser.parse_args()

with rosbag.Bag(options.output, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(options.input).read_messages():
        
        # If message has a header, change it
        if msg._has_header:
            msg.header.frame_id = options.frame

        # This topic has an array of which the headers must be changed
        if topic == '/dope/markers':
            for i in range(len(msg.markers)):
                msg.markers[i].header.frame_id = options.frame
        
        # Filter out data that is not needed
        if not (topic == '/rosout' or topic == '/rosout_agg' or topic.startswith('/dope/webcam')):
            print 'Saving topic %s' % topic
            outbag.write(topic, msg, t)
    