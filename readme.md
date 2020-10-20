# Saxion Smart Solution Semester - Bin Picking Robot

## Installation

Execute the following commands in the catkin workspace. (Probably `~/catkin_ws/src`)
Clone this repository:
```
git clone https://github.com/TripleSBinPicking/bin_picking_environment.git
```

Install Universal Robot URDF descriptions:
```
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
```

Install Robotiq URDF descriptions ([yes, the kinetic branch can be used for ROS melodic](https://github.com/ros-industrial/robotiq)):

A fork of the official ros-industrial repo is used, with some small changes for behaviour in Gazaebo.
```
git clone -b kinetic-devel https://github.com/TripleSBinPicking/robotiq.git
```

Update dependencies:
```
rosdep update
rosdep install --rosdistro melodic --ignore-src --from-paths src
```

Build catkin (in catkin workspace):
```
catkin_make
source ~/catkin_ws/devel/setup.bash
```

Your catkin workspace should now look like this:
```
catkin_ws
    |-build
    |-devel
    |-src
    |    |-bin_picking_environment
    |    |    |- onrobot_rg2
    |    |    |- triple_s_util
    |    |-robotiq
    |    |    |- ...
    |    |-universal_robot
    |    |    |- ...
```
## Usage

Command to the planning environment:
```
roslaunch triple_s_util planning_environment.launch
```
The following arguments can be used (append them to the last command):
 - `gripper:=rg2` load the Onrobot RG2 gripper instead of the Robotiq 2F 85 gripper
 - `sim:=false` use the actual robot instead of the simulation (not tested!!)
 - `camera_on_robot:=false` load a static camera instead of a camera on the robot
 - `world:=empty` load an empty world instead of the table world

Command to control the gripper (once previous command is running, replace `$DISTANCE` with a value):
```
rosservice call /control_robotiq "distance: $DISTANCE"
```
If the rg2 gripper is used the following command should be used
```
rosservice call /control_rg2 "distance: $DISTANCE"
```