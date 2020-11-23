"""
Author:       Niels de Boer
Date:         10-10-2020
Description:  DopeCollection can be used to collect data from DOPE that is received from the same image
"""
import rospy
import copy
import geometry_msgs.msg
import visualization_msgs.msg
import vision_msgs.msg
from .util import rosparamOrDefault

DEFAULT_OBJECT_NAMES = ['tomatosauce']

class DopeCollection:
    """ This class collects data for a single DOPE detection """
    def __init__(self):
        # Determines wether all data is received
        # Data is complete when the marker data is received.
        self.poses = []
        self.markers = None
        self.detection3DArray = None
        self.subscribers = {}
        self.total_poses = 0

        # DOPE pose Subscribers
        for object_to_locate in rosparamOrDefault('~object_names', DEFAULT_OBJECT_NAMES):
            self.total_poses += 1
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

        self.subscribers['detectionarray'] = rospy.Subscriber(
            rosparamOrDefault('~dope_detected_objects_topic', '/dope/detected_objects'),
            vision_msgs.msg.Detection3DArray,
            self.onDetectedObjectsReceived
        )

    def onDetectedObjectsReceived(self, detection3DArray):
        """
        Called when the detection 3D array is received

        detection3DArray -- object information (vision_msgs/Detection3DArray)
                            note that the contained source_cloud (sensor_msgs/PointCloud2) is empty
        """
        if not self.detection3DArray is None:
            rospy.logwarn('DopeCollection: Overwriting Detection3DArray! Was the collection properly cleared?')

        self.detection3DArray = detection3DArray

    def onPoseReceived(self, poseStamped, object_name):
        """
        Called when poses are received
        
        poseStamed -- pose (geometry_msgs.msg.PoseStamped)
        object_name -- the name of the object that this pose belongs to
        """
        if not self.isComplete():
            self.poses.append((object_name, poseStamped))
        else:
            rospy.logwarn('DopeCollection: Received poses while data collection is already complete!')

    def onMarkersReceived(self, markers):
        """
        Called when markers are received
        
        markers -- The markers (visualization_msgs.msg.MarkerArray)
        """
        if len(markers.markers) == 1 and markers.markers[0].action == visualization_msgs.msg.Marker.DELETEALL:
            # We don't do anything with the delete all marker
            pass
        else:
            if not self.markers is None:
                rospy.logwarn('DopeCollection: Overwriting markers! Was the collection properly cleared?')
            
            self.markers = markers
    
    def reset(self):
        """ Reset all the incoming messages to prepare for the next collection of messages """
        self.poses = []
        self.markers = None
        self.detection3DArray = None

    def getResults(self):
        """ Get the results of the DOPE analysis """
        return copy.deepcopy(self.poses), copy.deepcopy(self.markers), copy.deepcopy(self.detection3DArray)

    def isComplete(self):
        """ Check if all necessary data is received """
        return len(self.poses) >= self.total_poses \
            and not self.markers == None \
            and not self.detection3DArray == None