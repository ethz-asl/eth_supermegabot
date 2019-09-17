# Connect to robot

You can connect to the robot via ssh.
Make sure to install the ssh package on the robot first:
```
sudo apt update
sudo apt install openssh-server
```
Use ssh smb@11.0.0.5 to log into the robot.

Copy this alias to your .bashrc to connect to the robot's ros master (for running the opc)
```
alias connect-smb='export ROS_MASTER_URI=http://11.0.0.5:11311 ; export ROS_IP=`ip route get 11.0.0.5 | awk '"'"'{print $5; exit}'"'"'` ; echo "ROS_MASTER_URI and ROS_IP set to " ; printenv ROS_MASTER_URI ; printenv ROS_IP'
```