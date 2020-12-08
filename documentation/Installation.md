# Installation
This document will describe how to get this repository ready to run.
## Preparation steps
Make sure you have the following programs installed:
 - `git` 
 - `ros-melodic` (See the [ROS installation page](http://wiki.ros.org/ROS/Installation). This program was tested with `ros-melodic-desktop` on Ubuntu)
 - `python-pip` (Included with ROS)
 - [NVidia CUDA toolkit](https://developer.nvidia.com/cuda-10.2-download-archive) (installation instructions can be found [here](https://docs.nvidia.com/cuda/archive/10.2/cuda-installation-guide-linux/index.html), this repository is tested with version `10.2`)

## Configuring the workspace
1. Create a catkin workspace following the instructions [on the official ROS wiki](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Execute the following commands in a terminal in the catkin workspace. (Probably `~/catkin_ws/src`)
```bash
$ cd ~/catkin_ws/src
```

2. Clone this repository:
```
$ git clone --recurse-submodules https://github.com/TripleSBinPicking/bin_picking_environment.git
```

3. Go the the `bin_picking_environment/Deep_Object_Pose` directory and install the python modules:

```
$ pip install -r requirements.txt
```

4. Download all the files from [this](https://bit.ly/3S-ABWAC-WEIGHTS) folder and put it in `bin_picking_environment/Deep_Object_Pose/weights`

5. Go back to the the `~/catkin_ws` directory.
```
$ cd ~/catkin_ws
```

6. Install all the dependencies (and update your system):
```bash
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y
```

7. Build the system, and source the setup:
```bash
$ catkin_make
$ source devel/setup.bash
```

8. The file structure should look like this:
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

Read next:  
[Planning environment explanation](Planning%20Environment%20Explanation.md)