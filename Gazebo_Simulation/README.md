# Gazebo Drone Simulation
This project is the simulation interface with MAVROS of the Drone.

## System Requirement
- Ubuntu 18.04 or 16.04
- ROS Melodic or Kinetic
- Gazebo
- SITL (Software In The Loop) for Drone
- MAVROS (ROS package for interface with the drone)

## Gazebo Plugin Installation
The following plugin is a pure Gazebo plugin, so ROS is not needed to use it. You can still use ROS with Gazebo with normal gazebo-ros packages.
```
git clone https://github.com/khancyr/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
These instructions will clone the repository into the ardupilot_gazebo directory, and create a new build directory within it. The plugin will then be compiled and installed in the Gazebo plugin directory.

## Setting up SITL on Linux
### Clone ArduPilot repository
```
https://github.com/ArduPilot/ardupilot.git
```
### Install some required packages
If you are on a debian based system (such as Ubuntu or Mint), There is a script that will do it for you. From ardupilot directory :
```
cd ardupilot/Tools/environment_install/
./install-prereqs-ubuntu.sh -y
```
Reload the path (log-out and log-in to make permanent):
```
. ~/.profile
```

## MAVROS Installation
Replace `YOUR_ROS_DISTRO ` with `melodic` for Ubuntu 18.04 or `kinetic` for ubuntu 16.04.
```
sudo apt-get install ros-YOUR_ROS_DISTRO-mavros ros-YOUR_ROS_DISTRO-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```
For ease of use on a desktop computer, please also install RQT
```
sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins ros-kinetic-rqt-robot-plugins
```

## Setting up `drone-gazebo` package on your ROS workspace
Let's create and build a catkin workspace:
```
mkdir -p ~/catkin_ws/src
```
Then copy `drone-gazebo` in this repository to `~/catkin_ws/src` and do following command
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

## Start the Simulator
In a terminal window start Gazebo:
```
gazebo --verbose worlds/iris_arducopter_runway.world
```
In another terminal window, enter the ArduCopter directory and start the SITL simulation:
```
cd ardupilot/Tools/autotest/
./sim_vehicle.py -f gazebo-iris -v ArduCopter
```

## Connecting ROS with the Drone
In a terminal window run this command:
```
roslaunch drone-gazebo gazebo_mavros.launch
```

## First flight on simulation
```
----- run some scripts -----
```
