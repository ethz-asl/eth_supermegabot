## ETH Center for robotics summer school 2019

This is a basic set-up guide for the workspace packages and tutorial to use the super mega bots.

### Setting up your personal computer

We have a special workspace setup that should ensure that you can run everything that is possible on the robots also on your personal computer. Please follow the steps below:

## Install Ubuntu 18.04 LTS

Download the [desktop image](http://releases.ubuntu.com/18.04/).
Follow the installation instruction [here](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop).

Important: For the later installation of ROS you have to configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse.". You can follow the [Ubuntu guide for instructions](https://help.ubuntu.com/community/Repositories/Ubuntu) on doing this.

Update your installation to the newest version:
```
sudo apt update
sudo apt upgrade
```
## Preliminary dependencies:
We recommend you to use terminator, that allows you to have multiple terminals in one window.
It can be installed with.
```
sudo apt update
sudo apt install terminator
```
Install [git](https://www.atlassian.com/git/tutorials/what-is-git) and other dependencies:
```
sudo apt update
sudo apt-get install git python-catkin-tools doxygen 
sudo apt-get install ros-melodic-octomap ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-rosserial ros-melodic-joy ros-melodic-ompl ros-melodic-costmap-2d ros-melodic-velodyne-gazebo-plugins
sudo apt-get install libpcap0.8-dev libeigen3-dev libopencv-dev libboost-dev ros-melodic-cmake-modules libssh2-1-dev 
sudo apt-get install libglpk-dev
sudo apt-get install python-wstool net-tools
```
## Install ROS Melodic

Install ROS Melodic (recommended: “Desktop-Full Install”) following the [instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). We work with Catkin Command Line Tools (catkin build instead of catkin_make) to build
packages in your workspace. They can be installed with [apt-get](http://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get).

Setup your catkin workspace in which your packages will be built as follows.
Source the environment
```
source /opt/ros/melodic/setup.bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```
## Create and setup your catkin workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin init
catkin config --extend /opt/ros/melodic
catkin config --merge-devel
catkin config -DCMAKE_BUILD_TYPE=Release
```
We use wstool to manage packages in the workspace, at least for your initial setup, do:
```
cd ~/catkin_ws/src/
git clone https://github.com/ethz-asl/eth_robotics_summer_school_2019.git
wstool init
wstool merge eth_robotics_summer_school_2019/dependencies.rosinstall
wstool up
```
Build the workspace (This can take some time, grab a coffee or take a nap).
```
cd ~/catkin_ws/
catkin build
```
Source your workspace.
```
source devel/setup.bash
```
Add your workspace to the .bashrc such that it is sourced every time you start a new
shell (terminal).
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

# Troubleshooting  
* Header file not found:  
Solution: resource your catkin workspace, then build again.  
```
source ~/catkin_ws/devel/setup.bash  
catkin build  
```

* DCMAKE_BUILD_TYPE not specified, etc:  
Solution: Double check the catkin config  
```
catkin config  
```
Then you should find CMake args as below:  
Additional CMake Args: -DCMAKE_BUILD_TYPE=Release  

* Memory allocation (leak) issue:  
For compiling ocs2_ballbot_example or related examples needs 4G RAM per core.  
Solution: either limit the core while compiling by adding '-j1' or enable swap space.  
```
catkin build -j1
```

* No internet connection on robot:
Verify that you cannot ping google:
```
ping www.google.com  
```
If you do not get a connection, the 4G modom may have reconnected to a different sender. A power-cylcle on the Nighhawk router (The one with the 3 antennas) should fix this issue.

* VI Sensor does not respond:
Use the magic command:
```
sudo dhclient eno1
```
