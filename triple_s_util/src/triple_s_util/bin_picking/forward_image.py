"""
Author:       Niels de Boer
Date:         11-11-2020
Description:  Class that will forward images to another topic
"""
import rospy
import message_filters
import sensor_msgs

class ForwardImage:
    def __init__(self, input_camera_raw, input_camera_info, output_camera_raw, output_camera_info):
        """
        Forward single images (with camera info) to another topic.

        input_camera_raw -- The topic name of the raw camera input 
        input_camera_info -- The topic name of the camera info input
        output_camera_raw -- The topic name of the raw camera output
        output_camera_info -- The topic name of the camera info output
        """
        self._forwardImage = False

        # Init subscriber topics
        self._input_raw = message_filters.Subscriber(
            input_camera_raw,
            sensor_msgs.msg.Image
        )
        self._input_info = message_filters.Subscriber(
            input_camera_info,
            sensor_msgs.msg.CameraInfo
        )
        # Sync topics on time
        self._combined_topic = message_filters.TimeSynchronizer([self._input_raw, self._input_info], 1)
        self._combined_topic.registerCallback(self.onCameraFrameReceived)

        # Init publisher topics
        self._output_raw = rospy.Publisher(
            output_camera_raw,
            sensor_msgs.msg.Image,
            queue_size=4,
            latch=True
        )
        self._output_info = rospy.Publisher(
            output_camera_info,
            sensor_msgs.msg.CameraInfo,
            queue_size=4,
            latch=True
        )
    
    def onCameraFrameReceived(self, camera_raw, camera_info):
        """
        Called when a camera frame is received. Forward to another topic 
        if self.forwardImage is set to true.

        camera_raw -- The RGB data of the camera frame (sensor_msgs.msg.Image)
        camera_info -- The info of the camera (sensor_msgs.msg.CameraInfo)
        """
        if self._forwardImage:
            self._forwardImage = False
            self._output_raw.publish(camera_raw)
            self._output_info.publish(camera_info)
    
    def forwardNext(self):
        """ Forward the next image that is received """
        self._forwardImage = True

    def blockIndefinately(self):
        """ Block all images until forwardNext is called """
        self._forwardImage = False
