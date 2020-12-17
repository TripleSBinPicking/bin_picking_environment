# Selecting the specific object
Once DOPE has analyzed a frame of the camera, it must be determined which object is the best candidate to be picked up. This is done by the `/object_request` service. This document describes how the best candidate is determined.

## Methodology

[The data DOPE returns](Result%20from%20DOPE.md) is not all relevant for determining which object is the best candidate for picking up. Only the object id and pose are used. The confidence value of DOPE is not used, because DOPE already filters for a minimum confidence value. (See [Configuring DOPE - Inference Settings](Configuring%20DOPE.md#inference-settings))

First, all the detected objects that are not of the requested object type are filtered out. Then, for each object, the position is translated from the reference frame of the camera to the reference frame of the robot. This is needed in order to now the position of the object in relation to the robot. It is also determined if the object is within reach of the robot. This is done by defining an area in which the objects must lie.

Now that it is known were the objects lie with respect to the robot and that the objects are within reach of the robot, the object that is closest the the robot is regarded as the best candidate 