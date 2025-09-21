# Swarm Robotics for Exploration and Mapping
This repository contains the technical work related to the Final Year Project of our Mechatronics BTech program

### Team Members:
  - Jad Katerji
  - Lawrence Abou Karroum
  - Ghina Debian

### Technologies used: 
  - ROS2 Jazzy
  - Gazebo sim
  - Rasbperry PI 5

### Repo Content:
The way this repo is structured, we have a folder containing all code related to the simulation of our robot system, and a folder with all code related to the hardware implementation of our program.  

#### Trials_Code:  
  This folder contains many unrelated files each with code to control/process a specific hardware component that is included in our mobile robots (e.g. wheel encoder, wheel motors, lidar sensor, etc...). This is only used for testing purposes and is not included in the final program.
    
#### hardware_robots:  
This folder contains the ROS2 program that is implemented on the physical mobile robots. This package is intended to be running on the rasbperry pi 5 of each robot.  
Structur:  
The main package to be used is the hardware_software package. All other packages are controlling a specific feature either an algorithm (taken as an already existing package from the internet not our work, these are the same used for the simulations) or a hardware component.

***Note 1***: the ydlidar_ros2_driver package is a cpp package and not a ROS2 package, it requires different commands to build, so be careful when building it so that it can be properly used within the hardware_software package.

 ***Note 2***: we created python nodes to control/process hardware data from the rabperry pi directly. This is not the standard practice in ROS2 harwdare projects. The conventional way of going about it is adding an arduino and connecting it to the raspberry pi such that the arduino accesses the Rpi IOs, and cpp nodes are created to only interact with the arduino. the popular ros2_control package is used with this setup.

 ***Note 3***: in the source code files of hardware_software packages, there are some python files related to enabling mqtt communication between the robots for the purpose of sending their seperate map data to a device (whether it be a robot within the group or a web app) to be merged into one whole map. However, this functionality was not implemented in the final program and could be explored in future works. further investigation must be done to determine whether mqtt is the preferable method of communication over for example ros domains.

Launching complete robot program on mobile robots:  
note this is to be done on each robot individually

1. run python script to add namespace to nav2 param file before launching i.e.:  
  python3 preprocess_nav2_namespaced_yaml.py *namespace*

2. launch the robot_in_swarm_explore.launch.py file on the rpi of each individual mobile robot
     
   The following parameters can be specified:    
    - **namespace** (default: empty string) namespace to uniquely identify the topics and nodes of each robot
    - **map_merger_robot** (true or false. default: false) to specify if the robot will take the role of merging all the partial maps created by all the robots
    - **lidar_type** (ydlidar or rplidar. default: rplidar) if the robots has a rplidar or ydlidar

### Third-Party Packages
#### exploration and map merging alogorithms:
https://github.com/robo-friends/m-explore-ros2.git  
https://github.com/gingineer95/Multi-Robot-Exploration-and-Map-Merging/tree/main 
