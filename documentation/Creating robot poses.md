# Creating robot poses
This document describes how to create robot poses that can be used to move the robot to a fixed position.

### Step 1: Run the planning environment
Start up the planning environment:
```bash
$ roslaunch triple_s_util planning_environment.launch
```
Use all the paremeters that you need for this configuration. I.e. simulation, world, gripper etc.

### Step 2: Run the save_poses.py script
Run the script:
```bash
$ rosrun triple_s_util save_poses.py <file to save to>
```
For example, to place it in the `config/poses` directory of this repository, and name it `ur5.srdf`:
```bash
$ rosrun triple_s_util save_poses.py $(rospack find triple_s_util)/config/poses/ur5.srdf
```
Make sure the folder actually exists, otherwise the program won't run. 

### Step 3: follow program instructions
The program will first ask you for which move group it has to save positions. It will show you the available options.
You can type in the name of the group, or use the index the group has in the list (starting at zero of course).

After that the program will say:
```
Press enter if the robot is in the pose that has to be stored.
```
So, put your robot in the position you want to save, and hit enter. Only when you hit enter the position will be saved. Once it has fetched the position, it will ask you to give this position a name. Make sure this name is unique.

Finally, the program will ask you if you want to save another position. Insert `y` to enter another position, anything else will cause the program to save the file and exit.

### Step 4: Use the poses
The poses are now saved to the specified path in `srdf` format. They can be loaded in using the `poses` parameter of the [planning environment launchfile](Planning%20Environment%20Explanation.md#parameters).

Read next:  
[Creating a dataset using NNDS](NDDS%20tutorial.md)  