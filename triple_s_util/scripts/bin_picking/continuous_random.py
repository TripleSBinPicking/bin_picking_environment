#!/usr/bin/env python
"""
Author:       Niels de Boer
Date:         15-12-2020
Description:  Continuously request objects
"""
import rospy
import random
from triple_s_util.srv import PickupRequest, PickupRequestRequest
from triple_s_util.bin_picking.util import rosparamOrDefault

rospy.init_node('continuous_random', anonymous=True)

objects = rosparamOrDefault('/dope/class_ids', {}).keys()

service_name = rosparamOrDefault('/bin_picking/pick_up_request_service', '/pick_up_request')
rospy.wait_for_service(service_name)
service = rospy.ServiceProxy(service_name, PickupRequest)

while True:
    request = PickupRequestRequest()
    request.object_name = random.choice(objects)

    print 'Requesting object of type: %s' % request.object_name
    print service(request)

    rospy.sleep(2)