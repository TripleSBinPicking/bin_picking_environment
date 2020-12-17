# Running everything
In order to run all the necessary programs you can do two things. Either start all the launch files manually (everything can be configured as needed) or starting everything using a single launcfile. 

## Single launchfile
The simplest way to run everything is by using the [`run_simulation.launch`](../triple_s_util/launch/run_simulation.launch) launchfile. It can be started using the following command:
```bash
$ roslaunch triple_s_util run_simulation.launch run_dope:=<true|false>
```
The `run_dope` parameter determines wether DOPE is actually started, or wether fake object locations are used (as described in [Picking up object overview](Picking%20up%20object%20overview.md)).

The command will also start the simulation of the robot, rviz and the bin picking program.

Once everything is started you can start requesting objects!
```bash
$ rosservice call /pick_up_request "object_name: '<insert name of object here>'"
```
Remeber to place some objects in front of the camera if DOPE is enabled! This can be done manually in Gazebo by going to the _insert_ tab, or by running:

```bash
$ rosrun triple_s_util spawn_objects.py
```

## Starting manually
If you start every launchfile manually you have more control over the parameters that are used. The following commands are necessary:

```bash
$ roslaunch triple_s_util dope.launch
$ roslaunch triple_s_util planning_environment.launch
$ roslaunch triple_s_util object_grasping.launch
```

Every launchfile has it's own parameters that can be used. See [Picking up object overview](Picking%20up%20object%20overview.md) and [Planning environment explanation](Planning%20Environment%20Explanation.md) for more information.

Read next:  
[Running on another robot](Running%20on%20another%20robot.md)