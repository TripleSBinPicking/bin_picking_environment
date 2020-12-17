#!/usr/bin/env python
"""
Author:       Niels de Boer
Date:         15-12-2020
Description:  FakeFindObject starts a service which will return a random location for an object
"""
import rospy
import geometry_msgs.msg
from visualization_msgs.msg import MarkerArray, Marker
import triple_s_util.srv
from pyquaternion import Quaternion
import random

from triple_s_util.bin_picking.util import rosparamOrDefault

class FakeFindObject:
    def __init__(self):
        self.service = rospy.Service(
            rosparamOrDefault('/bin_picking/object_request_service', '/object_request'),
            triple_s_util.srv.ObjectRequest,
            self.onObjectRequest
        )

        self.object_dimensions = rosparamOrDefault('/dope/dimensions', {})
        self.object_meshes = rosparamOrDefault('/dope/meshes', {})
        self.object_colors = rosparamOrDefault('/dope/draw_colors', {})
        self.object_mesh_scale = rosparamOrDefault('/dope/mesh_scales', {})
        self.min_x_object = rosparamOrDefault('/bin_picking/min_x_object', -100)
        self.max_x_object = rosparamOrDefault('/bin_picking/max_x_object', 100)
        self.min_y_object = rosparamOrDefault('/bin_picking/min_y_object', -100)
        self.max_y_object = rosparamOrDefault('/bin_picking/max_y_object', 100)
        self.min_z_object = rosparamOrDefault('/bin_picking/min_z_object', -100)
        self.max_z_object = rosparamOrDefault('/bin_picking/max_z_object', 100)


        self.pose_publishers = {}

        for name, dimension in self.object_dimensions.items():
            self.pose_publishers[name] = rospy.Publisher(
                rosparamOrDefault('/bin_picking/dope_pose_topic_prefix', '/dope/pose_') + name,
                geometry_msgs.msg.PoseStamped,
                queue_size=10
            )
        
        self.marker_publisher = rospy.Publisher(
            rosparamOrDefault('/bin_picking/dope_markers_topic', '/dope/markers'),
            MarkerArray,
            queue_size=10
        )
        self.marker_id = 0


    def onObjectRequest(self, request):
        """
        Service callback
        
        request -- Object request

        returns -- position of the randomly generated object
        """
        result = triple_s_util.srv.ObjectRequestResponse()

        if not request.object_name in self.pose_publishers:
            rospy.logwarn('Object not detectable because it is not registered in DOPE!')
            result.found_object = False
            return result

        pose = self.getRandomPose()

        result.found_object = True
        result.object_pose = pose

        self.deleteAllMarkers()

        self.publishObjectMarker(request.object_name, pose)

        return result
    
    def getColorsForObject(self, object_name):
        """
        Get the color of an object. Gets the value either from the parameter server,
        or uses a gray-ish value

        object_name -- Name of the object of which the color is requested

        result -- cuboid size in array
        """
        if object_name in self.object_colors:
            result = [value / 255.0 for value in self.object_colors[object_name]]
            result.append(0.7)
            return result
        else:
            return [0.7] * 4

    def getMeshScaleForObject(self, object_name):
        """
        Get the mesh scale for an object. Gets the value either from the parameter server,
        otherwise scale is one.

        object_name -- Name of the object of which the mesh scale is requested

        result -- cuboid size in array
        """
        if object_name in self.object_mesh_scale:
            return self.object_mesh_scale[object_name]
        else:
            return 1
    
    def getCuboidSizeForObject(self, object_name):
        """
        Get the cuboid for an object. Gets the value either from the parameter server,
        or uses a 1m x 1m x 1m cuboid

        object_name -- Name of the object of which the cuboid is requested

        result -- cuboid size in array
        """
        if object_name in self.object_dimensions:
            return [value / 100 for value in self.object_dimensions[object_name]]
        else:
            return [1] * 3
    
    def deleteAllMarkers(self):
        """
        Delete all the previous markers
        """
        delete_markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_markers.markers.append(delete_marker)
        self.marker_publisher.publish(delete_markers)

    def getRandomPose(self):
        """
        Generate a random pose

        return -- geometry_msgs/PoseStamped
        """
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = rosparamOrDefault('/bin_picking/pose_reference_frame', 'base_link')
        
        pose.pose.position.x = random.uniform(self.min_x_object, self.max_x_object)
        pose.pose.position.y = random.uniform(self.min_y_object, self.max_y_object)
        pose.pose.position.z = random.uniform(self.min_z_object, self.max_z_object)

        quat = Quaternion.random()

        pose.pose.orientation.x = quat[1]
        pose.pose.orientation.y = quat[2]
        pose.pose.orientation.z = quat[3]
        pose.pose.orientation.w = quat[0]

        return pose

    def publishObjectMarker(self, object_name, pose):
        """
        Publish the marker for this object

        object_name -- Name of the object
        pose -- Pose of the object
        """
        marker = Marker()
        marker.header = pose.header
        marker.action = Marker.ADD
        marker.pose = pose.pose

        colors = self.getColorsForObject(object_name)

        marker.color.r = colors[0]
        marker.color.g = colors[1]
        marker.color.b = colors[2]
        marker.color.a = colors[3]

        marker.id = self.marker_id
        self.marker_id = self.marker_id + 1

        if object_name in self.object_meshes:
            marker.ns = "meshes"
            marker.type = Marker.MESH_RESOURCE
            
            scale = self.getMeshScaleForObject(object_name)

            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = scale
            marker.mesh_resource = self.object_meshes[object_name]
        else:
            marker.ns = "bboxes"
            marker.type = Marker.CUBE
            
            cuboid = self.getCuboidSizeForObject(object_name)

            marker.scale.x = cuboid[0]
            marker.scale.y = cuboid[1]
            marker.scale.z = cuboid[2]

        markers = MarkerArray()
        markers.markers.append(marker)

        self.pose_publishers[object_name].publish(pose)
        self.marker_publisher.publish(markers)

if __name__ == '__main__':
    rospy.init_node('fake_find_object', anonymous=True)
    
    rospy.loginfo('Initializing fake_find_object.py')

    FakeFindObject()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Exiting...')