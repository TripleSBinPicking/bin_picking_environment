# Saxion Smart Solution Semester - Bin Picking Robot
This repository is for a project for saxion' smart solution semester. It was carried out for the research group for mechatronics and VIRO. The goal of the project was to pick up specific objects from a bin and present them to the user.

The final solution was made for a [Universal Robots UR5 robot](https://www.universal-robots.com/products/ur5-robot/) and [ROS](https://www.ros.org/). A gripper and camera were mounted on the robot. The camera was used to detect the 6D positions (x-, y-, z-position and x-, y-, z-rotation) of objects. These object detections are performed using the [DOPE](https://github.com/NVlabs/Deep_Object_Pose) AI, created by NVidia. Once an object is detected, an algorithm is run to determine the optimal position for the gripper in order to pick up the object. Sadly, due to the COVID-19 pandemic, the system was only tested in simulations.

## Documentation
- [1. Robot setup](documentation/Installation.md)
  - [1.1 Installation](documentation/Installation.md)
  - [1.2 Planning Environment Explanation](documentation/Planning%20Environment%20Explanation.md)
  - [1.3 Connecting ROS to UR5](documentation/Connecting%20ROS%20to%20UR5.md)
  - [1.4 Creating robot poses](documentation/Creating%20robot%20poses.md)
- [2. DOPE setup](documentation/NDDS%20tutorial.md)
  - [2.1 NDDS tutorial](documentation/NDDS%20tutorial.md)
  - [2.2 Training DOPE](documentation/Training%20DOPE.md)
  - [2.3 Running DOPE on still images](documentation/DOPE%20on%20still%20images.md)
  - [2.4 Result from DOPE](documentation/Result%20from%20DOPE.md)
  - [2.5 Analyizing dope training](documentation/Analyzing%20DOPE%20training.md)
  - [2.6 Configuring DOPE](documentation/Configuring%20DOPE.md)
- [3. Picking up an object](documentation/Picking%20up%20object%20overview.md)
  - [3.1 Overview](documentation/Picking%20up%20object%20overview.md)
  - [3.2 Selecting the specific object](documentation/Selecting%20the%20specific%20object.md)
  - [3.3 Picking up the specific object](documentation/GrabPoseCaclulator.pdf)
  - [3.4 Configuring bin picking](documentation/Configuring%20bin%20picking.md)
- [4. Running everything](documentation/Running%20everything.md)
- [5. Running on another robot](documentation/Running%20on%20another%20robot.md)
- [6. Other](documentation/Pose%20estimation%20video.md)
  - [Create an annotated video](documentation/Pose%20estimation%20video.md)
- [7. Conclusions & Recommendations](documentation/Conclusions%20and%20Recommendations.md)
