## ETH robotics summer school 2019

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
## Install ROS Melodic

Install ROS Melodic (recommended: “Desktop-Full Install”) following the [instructions](http://wiki.ros.org/melodic/Installation/Ubuntu). We work with Catkin Command Line Tools (catkin build instead of catkin_make) to build
packages in your workspace. They can be installed with [apt-get](http://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get).

Setup your catkin workspace in which your packages will be built as follows.
Source the environment
```
source /opt/ros/melodic/setup.bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
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
sudo apt-get install liblapack-dev libblas-dev autotools-dev dh-autoreconf \
    libboost-all-dev python-setuptools cppcheck default-jre libgtest-dev \
    libglew-dev clang-format-3.9 python-git pylint python-termcolor \
    "ros-melodic-camera-info-manager*" protobuf-compiler protobuf-c-compiler \
    libssh2-1-dev libatlas3-base libnlopt-dev \
    "ros-melodic-tf2-*" python-pip python-autopep8 libreadline-dev ifstat \
    ntpdate sysstat libv4l-0 ros-melodic-gps-common
```
## (OPTIONAL) Install ccache for faster rebuilds.
ccache is a tool that caches intermediate build files to speed up rebuilds of the same code. On Ubuntu it can be set up with the following command. The max. cache size is set to 10GB and can be adapt on the lines below:

```bash
sudo apt install -y ccache &&\
echo 'export PATH="/usr/lib/ccache:$PATH"' | tee -a ~/.bashrc &&\
source ~/.bashrc && echo $PATH
ccache --max-size=10G
```
Your path (at least the beginning) should look like:
```
/usr/lib/ccache:/usr/local/cuda-5.5/bin/:/usr/lib/lightdm/lightdm:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games
```
And g++/gcc should now point to:
```
which g++ gcc
/usr/lib/ccache/g++
/usr/lib/ccache/gcc
```
Show cache statistics:
```
ccache -s
```
Empty the cache and reset the stats:
```
ccache -C -z
```
ccache only works for a clean workspace. You will need a `make clean` otherwise.

## Create and setup your catkin workspace.
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
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
## Create and setup your maplab workspace
```
mkdir -p ~/maplab_ws/src
cd ~/maplab_ws
catkin init
catkin config --extend ~/catkin_ws/devel/
catkin config --merge-devel
catkin config -DCMAKE_BUILD_TYPE=Release
```
Pull in the source code.
```
cd src
git clone https://github.com/ethz-asl/maplab_summer --recursive
git clone -b summer https://github.com/ethz-asl/maplab_dependencies --recursive
```
Build the project (this is where you can go for a second nap)
```
cd ~/maplab_ws
catkin build maplab
```
Finally add your workspace to the .bashrc such that it is sourced every time you start a new
shell (terminal).
```
echo "source ~/maplab_ws/devel/setup.bash" >> ~/.bashrc
```

# Connect to robot
Use ssh smb@11.0.0.5 to log into the robot.

Copy this alias to your .bashrc to connect to the robot's ros master (for running the opc)
```
alias connect-smb='export ROS_MASTER_URI=http://11.0.0.5:11311 ; export ROS_IP=`ip route get 11.0.0.5 | awk '"'"'{print $5; exit}'"'"'` ; echo "ROS_MASTER_URI and ROS_IP set to " ; printenv ROS_MASTER_URI ; printenv ROS_IP'
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
Solution: either limit the core while compiling by adding '-j1' or [enable swap space](https://bogdancornianu.com/change-swap-size-in-ubuntu/). (Use 16 blocks with 1GB each).
```
catkin build -j1
```

* No internet connection on robot:
Verify that you cannot ping google:
```
ping www.google.com  
```
If you do not get a connection, the 4G modem may have reconnected to a different sender. Try to
a) shut off and on the Nighhawk router (The one with the 3 antennas)
b) if that didn't fix it, reboot the robot with `sudo reboot 0`.

* VI Sensor crashes during startup:
Use the magic command (tab complete, the network interface name is different for every robot):
```
sudo dhclient enx[*]
```
The complete method for restarting VI-Sensor (thanks to @dwisth):

1. Close all ros process running.
2. Kill any existing DHCP clients.
`ps aux | grep dhclient`
`sudo kill -9 <process number>` as many times as required.
3. Reboot the VI sensor.
`ssh root@10.0.0.1`
`reboot`
wait approx 20 seconds. Check it has rebooted with `ping 10.0.0.1`.
3. Restart the DHCP client: `sudo dhclient enx<tab complete here>`. This takes time.
4. Start the launch files now.

* `fatal error: ethzasl_icp_mapper/LoadMap.h: No such file or directory`
Just `catkin build` again.

# Organisation
These tutorials are organised by the ETH Construction Robotics group.
