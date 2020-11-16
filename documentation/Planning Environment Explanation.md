# Planning Environment Explanation
_Last update: 16 november 2020_

This document describes what the [planning environment launchfile](../triple_s_util/launch/planning_environment.launch) does. The planning environment launchfile is the main launchfile for controlling the UR5 robot. It has the option to start and control the real robot, as well as a simulated robot.

## Starting the planning environment
Starting the planning environment is quite easy:
```
$ roslaunch triple_s_util planning_environment.launch
```

### Parameters
The launchfile has a few parameters. They can be used by adding `<parameter name>:=<parameter value>` to the launch command.

| Parameter | Valid values | Default | Explanation |
|---|---|---|---|
| `limited` | `true` or `false`  | `true` |Limit joint movement to `[-pi, pi]`. This makes it easier for MoveIt to plan movements, but sometimes causes weird movements |
| `sim` | `true` or `false` | `false` | Enable or disable the simulation. If the simulation is running, no real robot can be controlled |
| `paused` | `true` or `false` | `false` | Start the simulation paused |
| `gui` | `true` or `false` | `true` | Launch the Gazebo simulation UI (simulation only) |
| `robot_ip` | Ipv4 address | `172.16.0.10` | Ip address of the real robot (real robot only) |
| `camera_on_robot` | `true` or `false` | `true` | Launches the robot with the camera connected to the robot. If set to false, the camera is static in the world |
| `gripper` | `rg2`, `robotiq` or `none` | `rg2` | The gripper to put on the robot |
| `poses` | Path to `srdf` file | `triple_s_util/config/poses/test_sim.srdf` | Path to an `srdf` file that contains pose definitions |
| `environment` | `saxion` or `viro` | `saxion` | The environment to put the robot in |

## How it works
The launchfile uses lots of other launchfiles, which each have some use for the robot.

The first launchfile that is imported is [gazebo_ur5_gripper.launch](../triple_s_util/launch/gazebo_ur5_gripper.launch). This file is only loaded when a simulated robot is used, and it does the following things:
 - Startup an empty Gazebo world. [Gazebo](http://gazebosim.org/) is the simulation program that is used.
 - Load our robot configuration to the [ROS parameter server](http://wiki.ros.org/Parameter%20Server).
 - Spawn our robot configuration in Gazebo
 - Start the simulated robot controller
 - Start the simulated UR5 joint controllers
 - Start the simulated gripper controllers

If the real robot is used, and not a simulated robot. The following launchfile is first loaded: [physical_robot.launch](../triple_s_util/launch/physical_robot.launch). It does the following things:
 - Load our robot configuration to the [ROS parameter server](http://wiki.ros.org/Parameter%20Server).
 - Create a node that can publish the robot state
 - Launch the UR5 driver

Once a robot is running (either in a simulation, or a real robot), [MoveIt](https://moveit.ros.org/) is launched. MoveIt will plan all the movements of the robot.

A [semantic robot description](http://wiki.ros.org/srdf) is also loaded on the ROS parameter server. The semantic robot description contains joint groups, named robot poses and other configuration settings for the robot.

Finally, [rviz](http://wiki.ros.org/rviz) is started. Rviz can visualize what the robot is doing, and what is going to do.