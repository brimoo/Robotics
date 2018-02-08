# Robotics

Code for CSE180: Intro to Robotics

## Requirements

ROS is required to run this project. Installing ROS(kinetic or newer) on an Ubuntu system is recommended,
but it is possible to install on other operating systems.

## Building

Run `catkin_init_workspace` inside of the src directory and two folders will be created in the base 
workspace directory, build and devel. In order for ROS to know where your workspace is, you will either 
need to run `source devel/setup.bash` once per terminal, or put this command inside of your bashrc. 
For zsh users, the command is `source devel/setup.zsh`.

## Running

Due to the large degree of variance between each lab/project some are run differently than others. Below
are instructions to run each individual project/lab.

**Lab 1:** Simply use the launch file by enetring `roslaunch labpkg lab1.launch` from anywhere.

**Lab 2:** First enter `roslaunch husky_gazebo husky_empty_world.launch` to launch the simulator and then enter `rosrun labpkg square` and enjoy watching a robot move in a square(almost).
       
**Lab 3:** First enter`roslaunch husky_gazebo husky_playpen.launch` to launch the simulator, then enter `rosrun labpkbg getpose` and then enter `rosrun labpkg gotopose`. Ther terminal you entered the second command in will prompt you to enter a pose for the robot to move to. Do this and watch the robot attempt to move to the pose you select in the simulator.
