# Saxion Smart Solution Semester - Bin Picking Robot

## Installation
Make sure you have the following programs installed:
 - `git` 
 - `ros-melodic` (See the [ROS installation page](http://wiki.ros.org/ROS/Installation))
 - `python-pip`
 - [NVidia CUDA toolkit](https://developer.nvidia.com/cuda-10.2-download-archive) (installation instructions can be found [here](https://docs.nvidia.com/cuda/archive/10.2/cuda-installation-guide-linux/index.html), this repository is tested with version `10.2`)

### Installation steps

Execute the following commands in the catkin workspace. (Probably `~/catkin_ws/src`)
Clone this repository:
```
$ git clone --recurse-submodules https://github.com/TripleSBinPicking/bin_picking_environment.git
```

Go the the `bin_picking_environment/Deep_Object_Pose` directory and install the python modules:

```
$ pip install -r requirements.txt
```

Download the file `TomatoSoup.pth` from [this](https://drive.google.com/drive/folders/1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg) folder and put it in `bin_picking_environment/Deep_Object_Pose/weights`

Go the the `~/catkin_ws` directory.

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
    |    |    |- Deep_Object_Pose
    |    |    |- documentation
    |    |    |- onrobot_rg2
    |    |    |- robotiq
    |    |    |- triple_s_util
    |    |    |- universal_robot
    |    |    |- Universal_Robots_ROS_Driver
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
 - `environment:=<saxion|viro>` Set the environment to load. If the `viro` environment is loaded, make sure to copy all of the [table model files of Teams](https://teams.microsoft.com/_#/school/files/Automated%20Bin%20Picking%20with%20a%20Cobot?threadId=19%3A5da6c3f517af41d690e7a41124fc332f%40thread.tacv2&ctx=channel&context=VIRO%2520Table%2520Gazebo&rootfolder=%252Fteams%252Fo365-team050700-AutomatedBinPickingwithaCobot%252FShared%2520Documents%252FAutomated%2520Bin%2520Picking%2520with%2520a%2520Cobot%252FSolidworks%2520Models%252FVIRO%2520Table%2520Gazebo) (`Automated Bin Picking with a Cobot/Solidworks Models/VIRO Table Gazebo`) and place them in the `triple_s_util/meshes/viro` folder.

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

### [Create an annotated video](documentation/Pose%20estimation%20video.md)
This document describes how to make an annotated video.