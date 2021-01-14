# Conclusion
A general conclusion will first be given. After that specific conclusions for the object detection system and the object grasping system will be given.

## General
A system is build in ROS that is able to detect and pickup objects using an UR5 robot and a camera mounted on the robot. Using the DOPE AI the system is able to estimate the 6D pose of known object types using a camera. Once the positions of the object are known, the system will determine the object that is best suitable for picking up and calculate the best way this object can be picked up. A distinction is made between cubical and cylindrical objects.

## Object detection
The DOPE AI can recognize the objects it was trained on, although it has some problem detecting for example the peppermint roll if the label isn’t showing. This is a known problem with the AI and will always be a point on which it will score lower. Objects with a unique contour like the beer opener are easier to recognize because of that contour. Another problem with the AI currently is that it doesn’t handle shadows very well. This is likely because of less visible contours and dimmer colours and the usage of a relatively small dataset. This can be solved by adding a light source to the camera, improving the dataset and training for more hours.

The dataset is a good mix between synthetic and realistic data, and was used to train the AI to recognize the requested objects. Because of the slow training, the size of the dataset was kept relatively low, as not to impact the already heavy workload even more.  

Another factor that has limited the effectiveness of the AI, is the low number of epochs that could be trained for each object. Although the use of Kaggle was better than anything that could have been reached on our own hardware, it still was not as good as having a dedicated render computer that is available 24/7 with better processing power. 

## Object grasping
The hardcoded pick-up strategies used by the robot seem to work in simulation. It does require some predefinitions per unique item: both one axis and “cubic” or “cylindrical” class has to be defined. The axis is dependent on the orientation in the CAD software used. The “cubic” class is limited to objects that approximate a cuboid. When more complex shaped objects have to be picked up, another piece of code has to be written. 

The use of custom 3D printed inserts for the gripper claw decreased how precise the gripper has to be operated. However, when more very different objects have to be picked up, this advantage becomes less and less. When the objects are unknown, the claws can’t be tailored to suit the objects.

How well the pickup code works in the physical world is not known because access to the robot was not allowed anymore. 

# Recommendations
General recommendations will be given first. After that, specific recommendations for the object detections sytem and the object grasping system will be given.

## General
Picking up objects from a bin is not an easy task to do. Objects that lie close to the side of the bin often cannot be picked up by a gripper, because the gripper would have to move through the side of the bin. The bin forces the gripper to pick all objects from the top, while this is not always the ideal way to pick up something. A simpeler and probably more succesfull demo would be to place all the objects on a table, so the gripper can move freely without running into the side of a bin.

A user interface should be made to start and control the system (e.g. buttons to choose which object must be picked up). A user interface would allow a layman to use this system.

## Object detection
First is to make sure that items are used which have unique contours or have a recognizable colour palette. This will help the AI with recognizing the objects with, while requiring less training.

The dataset can still be improved. One example is to expand the number of images that the dataset consists of. This has been proven effective up to 100.000 images, after which the AI becomes saturated, although it is possible to make even larger datasets. However, this will not have as big as an effect as it has on the training time of the AI. Another way to improve the dataset might be to increase the resolution of the dataset. This has one problem in it making the dataset a lot bigger, which will increase training times. Another problem is that the training program only accepts 400x400 images as this is coded in. It might be possible to change this by editing the code of the training program itself. 

Training the AI on good hardware is also very much recommended, as this will speed up the training process. The original DOPE project used a workstation with 4 NVIDIA P100 graphics cards, while testing was done using a NVIDIA Titan X. Slower graphics cards can of course be used for training and running the AI, but it will take significantly longer. 

## Object grasping
It is recommended to test the grasp code in a physical setting as this is not done yet. It looks convincing in the simulations but has not been proven to work. If it is not working properly the code can be adapted accordingly, or another method of calculating the grasp can be used. 

A good alternative would be [GPD](https://github.com/atenpas/gpd). GPD uses the known meshes of the objects to calculate a list of possible grasp positions. A trained AI chooses the optimal grasp position from the list. This eliminates the need of predefining the objects. It can even be used to grasp unknown objects if the robot can make a proper point cloud. This can be done with a depth camera. However, this is a lot slower to run in comparison to the hard coded way, and on basic computers will decrease speed. 