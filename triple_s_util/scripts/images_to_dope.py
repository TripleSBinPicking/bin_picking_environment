#!/usr/bin/env python
#
# Author:       Niels de Boer
# Date:         04-11-2020
# Description:  Parse images to DOPE and let dope draw in the bounding boxes
# Usage:        roslaunch triple_s_util images_to_dope

import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import yaml

class ImagesToDope:
    def __init__(self):
        self.path_origin = rospy.get_param('~path_origin')
        self.path_save = rospy.get_param('~path_save')
        self.extension = rospy.get_param('~file_extension')
        
        self.bridge = CvBridge()
        self.received_images = 0

        self.dope_subscriber = rospy.Subscriber(rospy.get_param('~dope_topic_subscribe'), Image, self.imageReceived)
        self.dope_publisher = rospy.Publisher(rospy.get_param('~dope_topic_publish'), Image, queue_size=10, latch=True)
        self.dope_info_publisher = rospy.Publisher(rospy.get_param('~dope_topic_publish_info'), CameraInfo, queue_size=10, latch=True)

        # Load all filenames
        self.images = [f for f in os.listdir(self.path_origin) if os.path.isfile(os.path.join(self.path_origin, f)) and f.endswith(self.extension)]
        self.total_images = len(self.images)

        # Load the config for the camera
        self.yaml_config = yaml.load(file(os.path.join(self.path_origin, rospy.get_param('~camera_info_filename'))).read())

        # Start sending images
        self.sendNextImage()

        rospy.spin()

    def sendNextImage(self):
        """ Sends the first image in the list of images to the publisher, also deletes the image from the list """
        if len(self.images) > 0:
            self.current_image = self.images[0]
            del self.images[0]
            self.sendImage(self.current_image)
        else:
            print 'Finished!'

    def sendImage(self, image_name):
        """ Send an image to the publisher topic """
        image_file = cv2.imread(os.path.join(self.path_origin, image_name))

        image_msg = self.bridge.cv2_to_imgmsg(image_file, 'bgr8')
        
        print 'Publishing image: ', image_name
        
        self.yaml_config.header = image_msg.header
        self.dope_info_publisher.publish(self.yaml_config)
        self.dope_publisher.publish(image_msg)

    def imageReceived(self, image):
        """ Convert an sensor_msgs/Image to an .jpeg file """
        print 'Received an image'

        try:
            # Convert ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError, e:
            print(e)
        else:
            # Save OpenCV2 image as a jpeg
            image_path = os.path.join(self.path_save, self.current_image)

            if not cv2.imwrite(os.path.join(image_path), cv2_img):
                print 'Could not save image to ', image_path
            else:
                self.received_images = self.received_images + 1
                print 'Saved image as: ', self.current_image
                print 'Progress: %d/%d' % (self.received_images, self.total_images)
            
            self.sendNextImage()

if __name__ == '__main__':
    rospy.init_node('images_to_dope', anonymous=True)

    ImagesToDope()