#!/usr/bin/env python
# 
# Author:       Niels de Boer
# Date:         28-10-2020
# Description:  This script can be used to generate an srdf file containing poses of a robot.
#               This is done using moveit, so make sure that it is running.
#               Read documentation/Creating robot poses.md for instructions.
# Usage:        rosrun triple_s_util save_poses.py {filename}

import sys
import rospy
import moveit_commander

class SavePoses:
    def __init__(self, filename):
        """ Initialize the class, pass the filename in the parameter """
        self.filename = filename
        print("Connecting to moveit")
        
        moveit_commander.roscpp_initialize(sys.argv)

        # open the file
        self.file = open(self.filename, "w")

        # Get the robot commander
        self.robot = moveit_commander.RobotCommander()
        
        # Get the manipulator group to save
        self.group_name = self.askForGroupName()
        self.group_to_save = self.robot.get_group(self.group_name)

        # Get the joint names in the group
        self.joint_names = self.group_to_save.get_joints()

        # Dictionary of the saved positions
        self.joint_positions = {}

        # Recursive method that will keep asking for positions
        self.askForPosition()

        self.createFile()

    def askForGroupName(self):
        """ Ask for the group name that should be saved """
        possibilities = self.robot.get_group_names()

        # Keep doing this until the method returns something
        while True:
            chosen = raw_input("Which group would you like to save the positions for? %s: " % possibilities)

            if chosen in possibilities:
                return chosen
            elif chosen.isdigit():
                int_chosen = int(chosen)

                if int_chosen >= 0 and int_chosen < len(possibilities):
                    return possibilities[int_chosen]

            print "Please enter a valid value."

    def askForPosition(self):
        """ This method will keep fetching positions until the user is done """

        raw_input("Press enter if the robot is in the pose that has to be stored.")

        joint_values = self.group_to_save.get_current_joint_values()

        name = raw_input("Please enter the name of this pose: ")
        
        self.joint_positions[name] = joint_values

        if raw_input("Do you want to save another pose? [y/n]: ") == 'y':
            self.askForPosition()

    def createFile(self):
        """ Create the xml file. This could be done using a library,
            but because the xml file is quite simple it is done by concatenating strings. """
        result =  "<?xml version=\"1.0\"?>\n"
        result += "<robot name=\"save_poses\">\n"

        for position_name in self.joint_positions:
            result += "    <group_state name=\"" + position_name + "\" group=\"" + self.group_name +"\">\n"

            # Link the joint names to the position
            positions = dict(zip(self.joint_names, self.joint_positions[position_name]))

            for joint_name in positions:
                result += "        <joint name=\"" + joint_name + "\" value=\"" + str(positions[joint_name]) + "\" />\n"
        
            result += "    </group_state>\n"

        result += "</robot>"

        self.file.write(result)
        self.file.close()

        print "Saved to:", self.filename


if __name__ == '__main__':
    rospy.init_node('save_poses', anonymous=True)

    if len(sys.argv) == 2:
        SavePoses(sys.argv[1])
    else:
        rospy.logwarn("Path to file required!")
        sys.exit()