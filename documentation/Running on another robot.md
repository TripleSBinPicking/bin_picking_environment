# Running on another robot
This document describes what must be done in general in order to run this system on another robot than the UR5. Note that advanced knowlegde of ROS might be needed in order to complete these tasks.

## Robot configuration
### Robot description
A new `robot_description` should be created. The robot description is always loaded on the ROS parameter server and describes how the robot physically looks. An example of the robot description of the UR5 used in this repository can be found in [`triple_s_util/urdf/ur5.urdf.xacro`](../triple_s_util/urdf/ur5.urdf.xacro). Note that [`xacro`](http://wiki.ros.org/xacro) is used as the XML macro language in order to make it more readable for humans.

### Semantic robot description
The [semantic robot description](http://wiki.ros.org/srdf) (found in [`triple_s_util/config/ur5_gripper.srdf.xacro`](../triple_s_util/config/ur5_gripper.srdf.xacro)) should also be updated in order to match the new robot configuration.

## Launchfiles
Rather than editing the launchfiles in this repository it would be a better idea to create new launchfiles for your robot configuration. Some launchfiles can than be imported in your launchfles.

### Planning environment
The main launch file that should be updated for the new robot configuration is [`planning_environment.launch`](../triple_s_util/launch/planning_environment.launch). What this launcfile exactly does is described in [Planning environment explanation](Planning%20Environment%20Explanation.md), but in general it starts the simulation and robot control.

### Object grasping
The [object grasping launchfile](Picking%20up%20object%20overview.md#starting-bin-picking) is not dependend on the robot configuration and thus can be reused for other projects. Note that the configuration files that are loaded in this launchfile must be changed to match the robot configuration.