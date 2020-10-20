# Connecting ROS to UR5
This document explains the process of connecting the UR5 robot to an computer running ROS. This tutorial is written for using ROS-melodic and an UR5 from the CB3 series.

### Step 1: Configurig the UR5
TBD

### Step 2: Configure the ROS computer
Connect the UR5 directly (without a router) to a computer using an ethernet cable. In Ubuntu open settings and go to network settings. Click the `+` icon next to the ethernet options. If you have multiple ethernet adapters, make sure to pick the one the UR5 is connected to.

![Network Overview](resources/network_overview.png)

A window pops up to configure a new ethernet connection. Under the identity tab you can give the profile a name and make sure to select the MAC-address of the used ethernet adapter. Leave the cloned address and MTU fields untouched.

Go to the IPv4 tab and set it to manual. Enter a network address that matches the same netmask address of the UR5. So, if the IP address of the UR5 is `172.16.0.10` and the netmask is `255.255.255.0`, the first three bytes should match, and the last byte should be unique. In this case the address `172.16.0.1` is chosen. Also fill in the netmask. Leave all other options empty.

![Network IPv4](resources/network_ipv4.png)

Disable IPv6 on the IPv6 tab, and make sure no password is set on the Security tab. Now, you can click on apply. Make sure to activate the new profile by clicking on it.

You can test the connection by pinging the UR5. Open a terminal and type in the following command: `ping {IP of UR5}` (in our case `ping 172.16.0.10`) The result should be something along the lines of: `64 bytes from 172.16.0.10` repeated over an over again.

### Step 3: Launching the ROS programs
TBD