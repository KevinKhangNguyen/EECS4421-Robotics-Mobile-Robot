# EECS4421-Robotics-Mobile-Robot

Project: Mobile Robot

Group Members:
* Kevin Duong 217043936
* Kevin Nguyen 217229255
* Tom Vo 218799171
* Anagha Koroth 218819516

This project utilizes ROS2 and Gazebo to create and model an autonomous mobile robot in a virtual laboratory setting. 

This repository contains code associated with Chapter 5 and 4 of CPMR 3rd Editon by G. Dudek and M. Jenkin.
* Note: This repository only contains the specific files from cpmr_ch5 as part of our project. this requries the CPMR3 repository from https://github.com/YorkCFR/CPMR3

* aruco-laser-robot.launch.py - This provides a simulated aruco target and robot with both Lidar and Camera.

* aruco-laser-target.py - Main file for the robot mobility, aruco marker targeting, and lidar detection.

The aruco targets have to be manually imported via:

* 1. Making a directory for the ~/.gazebo/models
* 2. Copying the gazebo_models from this repository to the new directory of ~/.gazebo/models

The map used from the demo is included here. For now, it is intended to drag the cpmr_ch4 folder containing the map to the src folder of CPMR3.
