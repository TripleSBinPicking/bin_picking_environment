#!/usr/bin/env python

from __future__ import print_function
import rospy
from onrobot_rg2.srv import ControlRG2
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

MAX_ROTATION = 0.84
MIN_ROTATION = -0.30
MIN_MM = 0
MAX_MM = 110
LEFT_CONTROLLER_NAME = "/onrobot_rg2/rg2_left_controller/command"
RIGHT_CONTROLLER_NAME = "/onrobot_rg2/rg2_right_controller/command"

publisher_left = False
publisher_right = False

def get_rotation(distance):
    """ Calculate the rotation for a distance """
    return ((MAX_MM - float(distance) - MIN_MM) / MAX_MM) * (abs(MIN_ROTATION) + MAX_ROTATION) + MIN_ROTATION

def control_gripper(request):
    global publisher_left, publisher_right
    distance = request.distance

    if distance < MIN_MM or distance > MAX_MM:
        rospy.logwarn("%f is an invalid value. Should be between %f and %f", distance, MIN_MM, MAX_MM)
        return -1
    
    rotate_to = get_rotation(distance)

    msg_left = Float64MultiArray(data=[-rotate_to, -rotate_to, rotate_to])
    msg_right = Float64MultiArray(data=[rotate_to, rotate_to, -rotate_to])

    if publisher_right and publisher_left:
        publisher_right.publish(msg_right)
        publisher_left.publish(msg_left)

        send = "sec myProgram():\n"
        send += " textmsg(\"Hallo, wereld!\")\n"
        send += " RG2(measured_width+5, 40)\n"
        send += "end"

        pub_physical.publish(send)


        rospy.loginfo("Rotating to %f radians", rotate_to)
        return rotate_to
    else:
        rospy.logerror("Publishers have failed!")
        return -1

def init_server():
    global publisher_left, publisher_right, pub_physical
    rospy.init_node('control_rg2_server')
    service = rospy.Service('control_rg2', ControlRG2, control_gripper)

    publisher_left = rospy.Publisher(LEFT_CONTROLLER_NAME, Float64MultiArray, queue_size=3)
    publisher_right = rospy.Publisher(RIGHT_CONTROLLER_NAME, Float64MultiArray, queue_size=3)
    pub_physical = rospy.Publisher('/ur_hardware_interface/script_command', String, queue_size=10)

    rospy.loginfo("Ready to control the gripper")
    rospy.spin()

if __name__ == "__main__":
    init_server()