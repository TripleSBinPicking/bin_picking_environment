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

### Install manually

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
 - `gripper:=rg2` load the Onrobot RG2 gripper instead of the Robotiq 2F 85 gripper
 - `camera_on_robot:=false` load a static camera instead of a camera on the robot
 - `world:=empty` load an empty world instead of the table world

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

Start the planning environment (as in the simulation), but add the `sim:=false` option.