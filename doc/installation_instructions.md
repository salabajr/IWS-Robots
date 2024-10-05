# Installation Instructions

## System Setup

This package is designed for use with ROS Melodic running on Ubuntu 18.04.

## Install ROS

First ensure that ROS (robot operating system) is installed on your system. Follow these instructions to install ROS:

http://wiki.ros.org/melodic/Installation/Ubuntu

## Install ROS packages

- Create catkin workspace
    ```
    mkdir -p ~/catkin_ws/src
    ```

- Download UR3 packages

    ```
    cd ~/catkin_ws
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
    git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot 
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_controllers_cartesian.git src/Universal_Robots_ROS_controllers_cartesian
    ```

- Force Torque Sensor packages
    ```
    git clone https://github.com/UTNuclearRoboticsPublic/netft_utils.git src/netft_utils

    ```

- Leader/Follower packages

    ```
    git clone https://gitlab.nist.gov/gitlab/el-wireless/iws_leader_follower.git src/iws_leader_follower
    ```

## Install dependencies

Run the following commands to ensure that the proper dependencies are installed:

```
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Build the Packages

To build the packages use the command `catkin_make`

If you have multiple catkin workspaces be sure to source the setup file in every terminal window using the command `source ~/catkin_ws/devel/setup.bash`. If not you can add the line to `~/.bashrc`.

If everything builds correctly the system should be ready to use. Refer to the [running instructions](running_instructions.md) to run the software.