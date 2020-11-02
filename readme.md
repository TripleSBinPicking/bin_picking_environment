# Saxion Smart Solution Semester - Bin Picking Robot

## Installation
Make sure you have the following programs installed:
 - `git` 
 - `ros-melodic` (See the [ROS installation page](http://wiki.ros.org/ROS/Installation))

Execute the following commands in the catkin workspace. (Probably `~/catkin_ws/src`)
Clone this repository:
```
$ git clone https://github.com/TripleSBinPicking/bin_picking_environment.git
```

### Installation steps

Go back to the `catkin_ws/src` directory. Clone the following repositories for the Universal Robots driver, the Universal Robot robot descriptions and the Robotiq descriptions.

```bash
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
$ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git
$ git clone -b kinetic-devel https://github.com/TripleSBinPicking/robotiq.git
```

Install all the dependencies (and update your system):
```bash
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y
```

Build the system, and source the setup:
```bash
$ catkin_make
$ source devel/setup.bash
```

The file structure should look like this:
```
catkin_ws
    |-build
    |-devel
    |-src
    |    |-bin_picking_environment
    |    |    |- documentation
    |    |    |- onrobot_rg2
    |    |    |- triple_s_util
    |    |-fmauch_universal_robot
    |    |    |- ...
    |    |-robotiq
    |    |    |- ...
    |    |-Universal_Robots_ROS_Driver
    |    |    |- ...
```
## Usage

### Simulation
Command to the planning environment:
```
roslaunch triple_s_util planning_environment.launch
```
The following arguments can be used (append them to the last command):
 - `gripper:=<gripper name>` load another gripper (Either `robotiq`, `rg2` (default) or `none`)
 - `camera_on_robot:=<true|false>` load a static camera instead of a camera on the robot (defaults: `true`)
 - `poses:=<pose filename>` load poses from an srdf file (default: `$(find triple_s_util)/poses/test_sim.srdf`)
 - `paused:=<true|false>` Start the simulation paused (default: `false`)
 - `limited:=<true|false>` Limit joint movement to [`-pi`, `pi`] instead of [`-2pi`, `2pi`] (default: `true`)
 - `environment:=<saxion|viro>` Set the environment to load. If the `viro` environment is loaded, make sure to copy the table model files of Teams and place them in the `triple_s_util/meshes/viro` folder.

Command to control the gripper (once previous command is running, replace `$DISTANCE` with a value):
```
$ rosservice call /control_robotiq "distance: $DISTANCE"
```

If the rg2 gripper is used the following command should be used

```
$ rosservice call /control_rg2 "distance: $DISTANCE"
```

### Real robot

Follow the instructions in [this](documentation/Connecting%20ROS%20to%20UR5.md) document on how to set up the UR5.

Start the planning environment (as in the simulation), and use the following additional parameters:
 - `sim:=false` to disable the simulation and use the real robot
 - `robot_ip:=<ip of robot>` to configure the ip of the robot

Be aware that MoveIt! sometimes has issues with finding plans with full joint limits [`-2pi`, `2pi`] and that this program by default limits the joints to [`-pi`, `pi`]. This has as a consequence that if the real robot is starting in a position which is outside of these limits, MoveIt! will not be able to find any solution. To solve this, disable the joint limits by adding `limited:=false`, or by moving the robot in a valid position before starting the planning environment.

## Documentation
### [Connecting ROS to UR5](documentation/Connecting%20ROS%20to%20UR5.md)
This document describes how to setup the UR5 and ROS, so ROS can control the movement of the UR5

### [Creating robot poses](documentation/Creating%20robot%20poses.md)
This document describes how to create a file with all the robot poses.