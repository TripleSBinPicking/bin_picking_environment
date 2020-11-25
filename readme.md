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

Download all the files from [this](https://bit.ly/3S-ABWAC-WEIGHTS) folder and put it in `bin_picking_environment/Deep_Object_Pose/weights`

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
$ roslaunch triple_s_util planning_environment.launch
```
More information on the `planning_environment` launchfile can be found [here](documentation/Planning%20Environment%20Explanation.md).

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

## Documentation (WIP)
- [1. Robot setup](#installation)
  - [1.1 Installation](#installation)
  - [1.2 Connecting ROS to UR5](documentation/Connecting%20ROS%20to%20UR5.md)
  - [1.3 Creating robot poses](documentation/Creating%20robot%20poses.md)
- [2. DOPE setup]()
  - [2.1 NNDS tutorial](documentation/NNDS%20tutorial.md)
  - [2.2 Training DOPE](documentation/Training%20DOPE.md)
  - [2.3 Result from DOPE](documentation/Result%20from%20DOPE.md)
- [3. Usage](#usage)
  - [3.1 Planning Environment Explanation](documentation/Planning%20Environment%20Explanation.md)
- [2. Other](#installation)
  - [Create an annotated video](documentation/Pose%20estimation%20video.md)