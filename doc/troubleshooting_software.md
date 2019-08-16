## Troubleshooting
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