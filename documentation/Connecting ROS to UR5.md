# Connecting ROS to UR5
This document explains the process of connecting the UR5 robot to an computer running ROS. This tutorial is written for using ROS-melodic and an UR5 from the CB3 series.

This turial is only valid when configuring a computer running ROS and an UR5 that are directly connected to each other over an ethernet cable (without any devices in between). 
To configure the devices, two IPv4-addresses are needed. For this tutorial the following adresses are chosen:
 - `172.16.0.10` for the UR5
 - `172.16.0.1` for the computer running ROS

Notice they are both on the same subnet. 

### Step 1: Enabling a network connection on the UR5
On the startup screen on the UR5 choose *Setup Robot* > *Network*. Insert the IP address for the UR5 and set the subnetmask to `255.255.255.0`.

### Step 2: Configure the network on the ROS computer
Connect the UR5 directly (without a router) to a computer using an ethernet cable. In Ubuntu open settings and go to network settings. Click the `+` icon next to the ethernet options. If you have multiple ethernet adapters, make sure to pick the one the UR5 is connected to.

![Network Overview](resources/network_overview.png)

A window pops up to configure a new ethernet connection. Under the identity tab you can give the profile a name and make sure to select the MAC-address of the used ethernet adapter. Leave the cloned address and MTU fields untouched.

Go to the IPv4 tab and set it to manual. Enter a network address that matches the same netmask address of the UR5. So, if the IP address of the UR5 is `172.16.0.10` and the netmask is `255.255.255.0`, the first three bytes should match, and the last byte should be unique. In this case the address `172.16.0.1` is chosen. Also fill in the netmask. Leave all other options empty.

![Network IPv4](resources/network_ipv4.png)

Disable IPv6 on the IPv6 tab, and make sure no password is set on the Security tab. Now, you can click on apply. Make sure to activate the new profile by clicking on it.

You can test the connection by pinging the UR5. Open a terminal and type in the following command: `ping {IP of UR5}` (in our case `ping 172.16.0.10`) The result should be something along the lines of: `64 bytes from 172.16.0.10` repeated over an over again.

### Step 3: Install URCap for external control
[Download](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/resources/externalcontrol-1.0.4.urcap) the URCap file and save it to your disk. This tutorial is based on version `1.0.4` of the URCap.

Copy the file to the `/programs` folder on the UR5. This can be done by using a USB-stick, or by using `scp`:
```bash
$ scp externalcontrol-1.0.4.urcap root@172.16.0.10:/programs
```

The default password for root is `easybot` (remember to change this if the robot is connected to a network!).

Go to the welcome screen once again, and choose *Setup Robot* > *URCaps* and click the `+`-button. Select the `externalcontrol-1.0.4.urcap` file and press open. Restart the UR5 by clicking the restart button.

![URCap setup](resources/ur5_urcap_setup.png)

### Step 4: Creating a robot program

TBD: [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md).