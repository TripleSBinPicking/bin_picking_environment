#!/usr/bin/env python
#
# Author:       Niels de Boer
# Date:         04-11-2020
# Description:  Save a topic that publishes sensor_msgs/Image to individual image files
# Usage:        roslaunch triple_s_util video_to_images
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import yaml

class VideoToImages:
    def __init__(self):
        # Load the parameters from the parameter server
        self.path = rospy.get_param('~path')
        self.camera_raw_topic = rospy.get_param('~camera_raw_topic')
        self.camera_info_topic = rospy.get_param('~camera_info_topic')
        self.total_images = rospy.get_param('~total_images')

        self.stored_images = 0
        self.bridge = CvBridge()

        camera_raw = rospy.Subscriber(self.camera_raw_topic, Image, self.videoReceivedCallback)
        self.camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cameraInfoCallback)

        # Wait until all the required images are stored
        while self.stored_images < self.total_images:
            pass

        camera_raw.unregister()

        print 'Successfully stored %d images, exiting...' % self.stored_images

    def videoReceivedCallback(self, image):
        """ Convert an sensor_msgs/Image to an .jpeg file """
        try:
            # Convert ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save OpenCV2 image as a jpeg
            image_path = os.path.join(self.path, 'image%04d.jpeg' % self.stored_images)

            if not cv2.imwrite(os.path.join(image_path), cv2_img):
                print 'Could not save image to ', image_path
            else:
                self.stored_images = self.stored_images + 1

                if self.stored_images % 30 == 0:
                    print 'Stored %d images' % self.stored_images
    
    def cameraInfoCallback(self, cameraInfo):
        """ Store the camera info to a file, this is only done once """
        file_to_save = file(os.path.join(self.path, rospy.get_param('~camera_info_output')), 'w')
        
        if yaml.dump(cameraInfo, file_to_save):
            self.camera_info.unregister()
            print 'Saved camera info'
        
        file_to_save.close()

if __name__ == '__main__':
    rospy.init_node('video_to_images', anonymous=True)

    VideoToImages()