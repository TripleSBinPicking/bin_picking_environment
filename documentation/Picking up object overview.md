# Object picking overview
This document describes how to start the bin picking program.

## Starting bin picking
Starting the bin picking programs is simple, just enter the following command in a commandline:
```bash
$ roslaunch triple_s_util object_grasping.launch
```

This will start two scripts, both of whom are explained in the next section.

The launchfile has two parameters, they are explained in the table below.

| Parameter | Valid values | Default | Explanation |
|---|---|---|---|
| `use_fake_objects` | boolean | `false` | If enabled, a random position will be chosen as object location, instead of relying on data from DOPE |
| `cont_requests` | boolean | `false` | If enabled, it is not necessary to manually request for an object to be picked up. The system will randomly try to pick up one of the registered objects. (See [Configuring DOPE](Configuring%20DOPE.md)) |

### Starting a pick up sequence
In order for the system to start the picking up sequence, the `/pick_up_request` service must be used. It can be invoked using [C++](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29) or [python](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29) by creating a client node (see the linked tutorials on how to do this). It is also possible to use the terminal:
```bash
rosservice call /pick_up_request "object_name: '<insert name of object here>'"
```
Note that the object that is requested can be put in the `object_name` parameter. The object must be configured in DOPE in order to be able to pick up the object. Once the pick up sequence is done, the service will return three values:
 1. `found_object` shows wether an object was detected
 2. `picked_up_object` shows wether the object was picked up
 3. `error_message` contains the error message if either one of the above values is false.

## Scripts explained

### [`find_object.py`](../triple_s_util/scripts/bin_picking/find_object.py)
The find object script starts a [ROS service](http://wiki.ros.org/Services). This service can be called and has a single parameter: the name of the object that must be picked up. Once the service is finished it will return whether it has found an object of this type and where this object is located (as described below and in the [service file](../triple_s_util/srv/ObjectRequest.srv)).

```
string object_name
---
bool found_object
geometry_msgs/PoseStamped object_pose
```

If an object is requested, it sends a single frame of the camera feed to DOPE. DOPE doesn't constantly analyze the camera feed, because this takes a lot of processing power and is not needed for this system. Once DOPE has analyzed the image, the find object script will filter out the objects of the type that was requested. If there are multiple objects available, the script will determine which objects is the best candidate for picking up.

More details on how it is determined which objects are picked up read [TMP](#TODO).

### [`bin_picking_sequence.py`](../triple_s_util/scripts/bin_picking/bin_picking_sequence.py)

The bin picking sequence script is in charge of everything. It first starts the `/pick_up_request` service. Once the service is called. It does the following things:
 1. Move the robot arm so the camera mounted on the robot arm is above the bin.
 2. Open the gripper
 3. Use the `/object_request` service to get the location of the object that is best suited for picking up
 4. Calculate the approach pose and the grasping pose for this object at the location
 5. Move to the approach pose
 6. Move to the grasp pose
 7. Close gripper
 8. Move back to the approach pose
 9. Move to the drop off location

Details on how to call this service are described in [Starting a pick up sequence](#starting-a-pick-up-sequence)

More details on how the approach pose and grasp pose are calculated can be found in [TMP](#TODO).