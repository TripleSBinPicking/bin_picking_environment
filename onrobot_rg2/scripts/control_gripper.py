#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
from std_msgs.msg import Float64MultiArray

MAX_ROTATION = 0.84
MIN_ROTATION = -0.30
MIN_MM = 0
MAX_MM = 110
LEFT_CONTROLLER_NAME = "/onrobot_rg2/rg2_left_controller/command"
RIGHT_CONTROLLER_NAME = "/onrobot_rg2/rg2_right_controller/command"
NODE_NAME = "onrobot_rg2_control_gripper"

def get_rotation(distance):
    """ Calculate the rotation for a distance """
    return ((MAX_MM - float(distance) - MIN_MM) / MAX_MM) * (abs(MIN_ROTATION) + MAX_ROTATION) + MIN_ROTATION

def control_gripper(distance):
    """ Set the position of the gripper fingers to a certain distance """
    print("Trying to control the gripper to position: %s"%distance)
    
    publisher_left = rospy.Publisher(LEFT_CONTROLLER_NAME, Float64MultiArray, queue_size=10, latch=True)
    publisher_right = rospy.Publisher(RIGHT_CONTROLLER_NAME, Float64MultiArray, queue_size=10, latch=True)
    
    rospy.init_node(NODE_NAME, anonymous=True)

    if not rospy.is_shutdown():
        rotate_to = get_rotation(distance)

        msg_left = Float64MultiArray(data=[-rotate_to, -rotate_to, rotate_to])
        msg_right = Float64MultiArray(data=[rotate_to, rotate_to, -rotate_to])

        publisher_right.publish(msg_right)
        publisher_left.publish(msg_left)

        print("Rotating to: %f radians"%rotate_to)

        rospy.sleep(1) # Make sure the messages get actually send
    else:
        print("Rospy shutdown")

if __name__ == "__main__":
    # Parse the argument and call control_gripper
    if len(sys.argv) == 2:
        distance = 0

        if sys.argv[1] == "open":
            distance = 110
        elif sys.argv[1] == "closed":
            distance = 0
        else:
            try:
                distance = float(sys.argv[1])
            except ValueError:
                print("Argument should be an float")
                sys.exit(1)

        if distance >= 0 and distance <= 110:
            control_gripper(distance)
        else:
            print("Distance should be between 0 and 110 mm.")
            sys.exit(1)
    else:
        print("Usage:")
        print("\tcontrol_gripper.py {distance}")
        print("\t\tControl the distance between the two fingers. Minimum 0 mm, maximum 110 mm")
        print("\tcontrol_gripper.py open")
        print("\t\tOpen the gripper completely (110 mm)")
        print("\tcontrol_gripper.py close")
        print("\t\tClose the gripper completely (0 mm)")
        sys.exit(1)

