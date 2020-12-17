# Configuring bin picking
The bin picking program can be configured as needed. This document describes how this can be done and what can be configured.

## Configuration file
The configuration file is formatted in [YAML](https://en.wikipedia.org/wiki/YAML). The default configuration that is loaded is located in [`triple_s_util/config/bin_picking.yaml`](../triple_s_util/config/bin_picking.yaml).

### Service settings
The following settings determine what services are used.

| Parameter | Valid values | Default | Explanation |
| --- | --- | --- | --- |
| `object_request_service` | ROS service name | `/object_request` | The name of the service to request an object pose |
| `pick_up_request_service` | ROS service name | `/pick_up_request` | The name of the service to request the pick up of an object | 
| `gripper_service` | ROS service name | `/control_rg2` | The service name to control the gripper state |

### Frame settings
The following settings are for configuring reference and planning frames.
| Parameter | Valid values | Default | Explanation |
| --- | --- | --- | --- |
| `pose_reference_frame` | Reference frame name | `base_link` | The reference frame the poses must be transformed to to be in the robots reference frame. |
| `manipulator` | Joint group name | `manipulator` | The joint group for which motion plannings will be created |
| `camera_link` | Reference frame name | `camara_sim_link` | The reference frame of the camera |

### Object pose filter
The following settings are for filtering the object position.
| Parameter | Valid values | Default | Explanation |
| --- | --- | --- | --- |
| `min_x_object` | Float | `-0.10` | The minimum x value the object should have |
| `man_x_object` | Float | `0.40` | The maximum x value the object should have |
| `min_y_object` | Float | `-0.50` | The minimum y value the object should have |
| `max_y_object` | Float | `-0.30` | The maximum y value the object should have |
| `min_z_object` | Float | `0.005` | The minimum z value the object should have |
| `max_z_object` | Float | `0.20` | The maximum z value the object should have |

### DOPE settings
The following settings are for the topics that communicate with DOPE.

| Parameter | Valid values | Default | Explanation |
| --- | --- | --- | --- |
| `dope_markers_topic` | ROS topic name | `/dope/markers` | The topic DOPE publishes markers on |
| `dope_pose_topic_prefix` | ROS topic name prefix | `/dope/pose_` | The topic prefix that DOPE uses for the POSE publishing |
| `dope_detected_objects_topic` | ROS topic name | `/dope/detected_objects` | The topic DOPE publishes all the results on |
| `dope_camera_raw` | ROS topic name | `/dope/webcam/image_raw` | The topic DOPE listens to for camera images |
| `dope_camera_info` | ROS topic name | `/dope/webcam/camera_info` | The topic DOPE listens to for camera info (should be synchronized with `dope_camera_raw`) |

### Pick up settings
The following settings are for determining the approach pose and grasp pose.
| Parameter | Valid values | Default | Explanation |
| --- | --- | --- | --- |
| `rodrigues_z_limit` | Value between `0` and `1` | `0.8` | The maximum absolute z value of an cylindrical object for it to be picked up using the rodrigues method |
| `cylindrical_axis` | List of objects that are cylindrical around a certain axis | ```tomatosauce: 'x'  ViroPeppermint: 'y'  ViroPen:'x'``` | The objects that are cylindrical must be put here, along with the axis they are cylindrical on |
