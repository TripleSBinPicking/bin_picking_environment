#!/bin/bash

cd ..

# Clone the Universal Robots Driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git

# Clone the robotiq descriptions
git clone -b kinetic-devel https://github.com/TripleSBinPicking/robotiq.git

# Clone the descriptions for the UR robots
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git

# Install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the workspace
catkin_make

source devel/setup.bash