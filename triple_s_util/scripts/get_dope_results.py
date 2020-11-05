#!/usr/bin/env python
#
# Author:       Niels de Boer
# Date:         05-11-2020
# Description:  Send an image to DOPE
import rospy
import sys
import cv2
import yaml
import argparse
from cv_bridge import CvBridge
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String


class ImageToDope:
    def __init__(self):
        rospy.init_node('get_dope_results', anonymous=True)

        self.bridge = CvBridge()
        # Image publish topics
        self.dope_image_publisher = rospy.Publisher('/dope/webcam/image_raw', Image, queue_size=10, latch=True)
        self.dope_info_publisher = rospy.Publisher('/dope/webcam/camera_info', CameraInfo, queue_size=10, latch=True)
        
        # Subscribe to the single topics
        self.dope_detected_objects = rospy.Subscriber('/dope/detected_objects', Detection3DArray, self.dopeDetectedObjects)
        self.dope_markers = rospy.Subscriber('/dope/markers', MarkerArray, self.dopeMarkers)

        # Lists for publishers
        self.dope_poses = []
        self.dope_dimensions = []
        
        self.img_out = None
        self.dope_rgb_points = None

        # For each loaded modal two topics are created by DOPE:
        # /dope/pose_{model name} outputs markers for the position
        # /dope/dimension_{model name} outputs dimensions
        all_topics = rospy.get_published_topics('/dope')
        
        for object_pose_topic in [topic[0] for topic in all_topics if topic[0].startswith('/dope/pose_')]:
            self.dope_poses.append(rospy.Subscriber(object_pose_topic, PoseStamped, self.dopeObjectPose, callback_args=object_pose_topic))

        for dimension_topic in [topic[0] for topic in all_topics if topic[0].startswith('/dope/dimension_')]:
            self.dope_dimensions.append(rospy.Subscriber(dimension_topic, String, self.dopeDimension, callback_args=dimension_topic))

        print('Subscribed to all dope topics. Waiting for results.')

    def dopeDetectedObjects(self, detection3DArray):
        """ Print the Detection3DArray message """
        print('detected_objects:')
        # Not important for now, only clutters the commandline
        # print(detection3DArray)
    
    def dopeMarkers(self, markers):
        """ Print the MarkerArray """
        print('markers:')
        # Only the first marker is printed, to keep it clean
        print(markers.markers[0])
    
    def dopeRgbPoints(self, image):
        """ If enabled, saving the resulting image """
        print('Image received')

        if self.img_out:
            image_file = self.bridge.imgmsg_to_cv2(image, 'bgr8')
            cv2.imwrite(self.img_out, image_file)
            print('Image stored')


    def dopeObjectPose(self, poseStamped, objectType):
        """ Print the pose """
        print('Pose for %s: ' % objectType)
        # print(poseStamped)

    def dopeDimension(self, string, objectType):
        """ Print the dimensions of the object """
        print('Dimensions for %s: ' % objectType)
        print(string)

    def sendImageToDope(self, image, info):
        """ Send an individual image to DOPE 
            image : sensor_msgs/Image
            info : sensor_msgs/CameraInfo
        """
        info.header = image.header
        self.dope_info_publisher.publish(info)
        self.dope_image_publisher.publish(image)
        print('Image send')
    
    def sendImageFromPathToDope(self, path_img, path_config):
        """ Send an image from a path to DOPE """
        image_file = cv2.imread(path_img)
        image_msg = self.bridge.cv2_to_imgmsg(image_file, 'bgr8')

        info = yaml.load(file(path_config).read())

        print('Sending image from path: %s' % path_img)
        print('With config: %s' % path_config)

        self.sendImageToDope(image_msg, info)
    
    def setOutputImage(self, output_image):
        """ Enable outputting the DOPE image """
        self.img_out = output_image

        if not self.dope_rgb_points:
            self.dope_rgb_points = rospy.Subscriber('/dope/rgb_points', Image, self.dopeRgbPoints)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Show results of dope in the commandline')
    parser.add_argument('--input-image', help='Path to image to send to DOPE', dest='input_image_path')
    parser.add_argument('--input-config', help='Path to camera config to send to DOPE', dest='input_config_path')
    parser.add_argument('--output-image', help='Path to save the output image', dest='output_image')
    parsed = parser.parse_args()

    itd = ImageToDope()

    if parsed.output_image:
        itd.setOutputImage(parsed.output_image)

    if parsed.input_image_path and parsed.input_config_path:
        itd.sendImageFromPathToDope(parsed.input_image_path, parsed.input_config_path)

    rospy.spin()