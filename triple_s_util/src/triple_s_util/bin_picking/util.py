"""
Author:       Niels de Boer
Date:         10-10-2020
Description:  Contains utility methods for bin_picking
"""
import rospy

def rosparamOrDefault(param, default_value):
    """
    Get a value from the ROS parameter server if it exist.
    If it does not exist, return the default_value. 

    param -- The name of the parameter on the parameter server
    default_value -- The value to return when the parameter is not on the parameter server
    """
    if rospy.has_param(param):
        return rospy.get_param(param)
    else:
        return default_value