"""
Author:       Niels de Boer
Date:         10-10-2020
Description:  DopeCollection can be used to collect data from DOPE that is received from the same image
"""
import rospy
import copy
import geometry_msgs.msg
import visualization_msgs.msg
from .util import rosparamOrDefault

DEFAULT_OBJECT_NAMES = ['tomatosauce']

class DopeCollection:
    """ This class collects data for a single DOPE detection """
    def __init__(self):
        # Determines wether all data is received
        # Data is complete when the marker data is received.
        self.is_complete = False
        self.poses = []
        self.markers = None
        self.subscribers = {}

        # DOPE pose Subscribers
        for object_to_locate in rosparamOrDefault('~object_names', DEFAULT_OBJECT_NAMES):
            rospy.loginfo('Subscribing to poses for objects of type "%s"' % object_to_locate)
            self.subscribers['dope_pose_%s' % object_to_locate] = rospy.Subscriber(
                rosparamOrDefault('~dope_pose_topic_prefix', '/dope/pose_') + object_to_locate,
                geometry_msgs.msg.PoseStamped,
                self.onPoseReceived,
                callback_args=object_to_locate
            )

        self.subscribers['marker'] = rospy.Subscriber(
            rosparamOrDefault('~dope_markers_topic', '/dope/markers'),
            visualization_msgs.msg.MarkerArray,
            self.onMarkersReceived
        )

        print self.subscribers

    def onPoseReceived(self, poseStamped, object_name):
        """
        Called when poses are received
        
        poseStamed -- pose (geometry_msgs.msg.PoseStamped)
        object_name -- the name of the object that this pose belongs to
        """
        if not self.is_complete:
            self.poses.append((object_name, poseStamped))
        else:
            rospy.logwarn('Received poses while data collection is already complete!')

    def onMarkersReceived(self, markers):
        """
        Called when markers are received
        
        markers -- The markers (visualization_msgs.msg.MarkerArray)
        """
        if len(markers.markers) == 1 and markers.markers[0].action == visualization_msgs.msg.Marker.DELETEALL:
            # We don't do anything with the delete all marker
            pass
        else:
            self.markers = markers
            self.is_complete = True
    
    def reset(self):
        """ Reset all the incoming messages to prepare for the next collection of messages """
        self.poses = []
        self.markers = None
        self.is_complete = False

    def getResults(self):
        """ Get the results of the DOPE analysis """
        return copy.deepcopy(self.poses), copy.deepcopy(self.markers)