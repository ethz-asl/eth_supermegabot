Slides and files can be found here:
https://drive.google.com/open?id=1wGgP7p-89cBF9sVolPy8bBC55QXuZTET

Update repos:
`cd ~/catkin_ws/src && wstool_up`

Install dependencies: `sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev ros-melodic-ddynamic-reconfigure-python git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev`

Set up access rights:
```
sudo usermod -a -G video smb
sudo usermod -a -G dialout smb
cd ~/catkin_ws/src/librealsense
./scripts/setup_udev_rules.sh
udevadm control --reload-rules && udevadm trigger
```

Build the realsense stack:
`catkin build realsense_eth_robotics_summer_school_2019`

Run the realsense:
`roslaunch realsense_eth_robotics_summer_school_2019 realsense.launch`
