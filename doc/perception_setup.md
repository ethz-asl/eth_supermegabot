## Setup and run the perception stack only

Build the required packages:
```
cd ~/catkin_ws/
catkin build smb_confusor
catkin build ethzasl_icp_mapper
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

For running only the state estmiation and SLAM, e.g., on bag-files, you can use the following launch files:
**This replaces the launching of smb_state_estimator.launch with smb_confusor.launch.**
```
roslaunch smb_confusor smb_confusor.launch
roslaunch ethzasl_icp_mapper supermegabot_dynamic_mapper.launch
```
