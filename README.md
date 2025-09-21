# Swarm Robotics for Exploration and Mapping
This repository contains the technical work related to the Final Year Project of our Mechatronics BTech program at the University of Balamand

### Team Members:
  - Jad Katerji  
    https://github.com/jad-katerji  
    jadkaterjio@gmail.com
    
  - Lawrence Abou Karroum  
    https://github.com/Lawrenceak  
    lawrenceaboukarroum1@gmail.com
    
  - Ghina Debian  
    https://github.com/GhinaDebian  
    ghinadebian@gmail.com  

### Technologies used: 
  - ROS2 Jazzy
  - Gazebo sim
  - Rasbperry PI 5

### Repo Content:
The way this repo is structured, we have a folder containing all code related to the simulation of our robot system, and a folder with all code related to the hardware implementation of our program.  

#### simulation_robots:  
This folder contains a multi-robot mapping simulation program using the turtlebot3 package and Gazebo sim simulation software.  

to run the complete program, launch multi_slam_nav.launch.py file in leader_follower package

this will open an rviz window to display the maps that are being built by each robot individually, and a window to display the merged map of all the individual ones in real time. No user input is needed while program is running, the robots will be autonomously exploring the environment.  
other launch files are included as well, these implement different parts of the functionality of the complete program (e.g  single robot exploration, multi robot mapping but user must move the robots with teleop commands, multi robot mapping but user must move the robots by calling the Nav2 navigateToPose action server, also some modification in the tf frames of each robot relative to eachother, etc...)

<img width="2748" height="1736" alt="2 turtlebot3 robots in turtlebot3 house" src="https://github.com/user-attachments/assets/17915367-9329-4bc6-984d-37cda618eac4" />
<img width="2748" height="1736" alt="2 turtlebot3 robots in turtlebot3 house with lidar scan on" src="https://github.com/user-attachments/assets/e929ab0d-a3f7-4bf2-993b-348f418e1aad" />
<img width="2748" height="1736" alt="merged map" src="https://github.com/user-attachments/assets/72538e9a-b809-47f6-8cbf-77f6b23571b3" />


## TODO:
- Make program modular. I.e. we should be able to input the number robots we want, with their names and poses, and the code would take care of launching the required nodes for each. 

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
  <br>
  
  ![mobile_robots](https://github.com/user-attachments/assets/588d4bf9-e045-4115-bc5f-63c4d4a749ce)


<br>

## TODO:
- Implement a multi-robot exploration algorithm.  <br>
<br>
  The program we have right now runs a single-robot exploration algorithm for each robot, this means the robots' decisions for where to move is completely independant of the positions of the other robots. In other words the robots are not aware of eachother in their envornment. This is obviously not in alignment with the concept of swarm robotics as the exploration algorithm does not create a swarm intelligence. Our work can be considered a foundation, a stepping stone, to further implement swarm algorithms to the project.
  
<br>

### Third-Party Packages
#### exploration and map merging alogorithms:
https://github.com/robo-friends/m-explore-ros2.git  
https://github.com/gingineer95/Multi-Robot-Exploration-and-Map-Merging/tree/main 

#### turtlebot3 robot simulation:  
https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

#### navigation:  
https://github.com/ros-navigation/navigation2.git
