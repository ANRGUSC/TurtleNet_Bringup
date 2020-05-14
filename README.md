# TurtleNet

TurtleNet is a project out of the Autonomous Networks Research Group at the University of Southern California investigating multi-robot exploration and mapping via ultrawide-band trilateration.

See also:   
https://github.com/ANRGUSC/TurtleNet  
https://github.com/ANRGUSC/gmapping_and_TEAM  

Our testbed is comprised of several Turtlebot3 Burgers integrated with Pozyx Anchors.

Turtlebot3 Burgers  
specs: http://emanual.robotis.com/docs/en/platform/turtlebot3/specifications/#specifications  
source code: https://github.com/ROBOTIS-GIT/turtlebot3  

Pozyx Anchor  
specs: https://www.pozyx.io/shop/product/creator-anchor-69  
python library: https://pypozyx.readthedocs.io/en/develop/  

### This repository

This repository contains the code which runs on the robot, interfaces with the UWB radio, and performs trilateration.

To use this repository, install the turtlebot3 source code as instructed here: http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/

Then replace the turtlebot3_bringup package in your catkin workspace with this repository and re-build your workspace.

The `in_sim` branch should be used when testing code in Gazebo. The `on_robot` branch should be used when testing code on the robots.

#### Launch
General:
`turtlebot3_pozyx.launch` launches the pozyx ROS node.

For 4 robots:
`tb3_x.launch` is comparable to `turtlebot3_robot.launch` under the `tb3_x` namespace (the namespace of robot x) and establishes the other three robots as instantaneous anchors for trilateration.

#### Scripts
`pozyx.py` contains the pozyx ROS node
`log_position.py` is used for evaluation localization, but should only be used with Gazebo

To simulate TurtleNet in Gazebo, download our TurtleNet_Simulations package here:  
https://github.com/ANRGUSC/TurtleNet_Simulations
