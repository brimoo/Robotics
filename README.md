# Robotics

Code for CSE180: Intro to Robotics

## Requirements

ROS is required to run this project. Installing ROS(kinetic or newer) on an Ubuntu system is recommended,
but it is possible to install on other operating systems.

## Building

 Run `catkin_make` inside of the base workspace directory (CSE180robotics) and two folders will be created, build and devel.
 In order for ROS to know where your workspace is, you will either need to run `source CSE180robotics/devel/setup.bash` once
 per terminal, or put this command inside of your bashrc. For zsh users, the command is source CSE180robotics/devel/setup.zsh'
 
## Running

To run each ROS project, use the roslaunch command as shown: `roslaunch <pkg_name> <launchfile>`

Example: `roslaunch labpkg lab1.launch'
` 
