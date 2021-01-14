# Conclusion
A general conclusion will first be given. After that specific conclusions for the object detection system and the object grasping system will be given.

## General
A system is build in ROS that is able to detect and pickup objects using an UR5 robot and a camera mounted on the robot. Using the DOPE AI the system is able to estimate the 6D pose of known object types using a camera. Once the positions of the object are known, the system will determine the object that is best suitable for picking up and calculate the best way this object can be picked up. A distinction is made between cubical and cylindrical objects.

## Object detection
`<tbd>`

## Object grasping
`<tbd>`

# Recommendations
General recommendations will be given first. After that, specific recommendations for the object detections sytem and the object grasping system will be given.

## General
Picking up objects from a bin is not an easy task to do. Objects that lie close to the side of the bin often cannot be picked up by a gripper, because the gripper would have to move through the side of the bin. The bin forces the gripper to pick all objects from the top, while this is not always the ideal way to pick up something. A simpeler and probably more succesfull demo would be to place all the objects on a table, so the gripper can move freely without running into the side of a bin.

A user interface should be made to start and control the system (e.g. buttons to choose which object must be picked up). A user interface would allow a layman to use this system.

## Object detection

## Object grasping