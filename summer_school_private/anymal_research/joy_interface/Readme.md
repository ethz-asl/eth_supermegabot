# Joystick interface

This package contains a flexible interface between generic linux joystick 
drivers and the joy\_manager.

## Overview

The package is divided into the joy\_interface\_node that generates 
AnyJoy msgs from a sensor\_msgs/Joy msg and the feedback\_node which transforms 
notifications to force feedback events. Launch files are provided for the 
**Logitech F710 Joypad**, the **X-Box 360 Pad Remote Controller** and the 
**3Dconnexion SpaceNavigator 6DOF joystick**.

## Plugins

To customize interface usage a plugin mechanism allows for dynamic loading of 
interface modules. Same as described in the the 
[joy\_manager](../joy_manager/Readme.md) ROS [pluginlib] packages which inherit 
from the **InterfaceModuleBase** can preprocess the incomming [sensor_msgs/Joy] 
messages.

## udev 
To allow the automated detection of the currently used joystick copy the 
contents of the udev folder to your local udev-rules directory:

**commands**:

```bash
roscd joy_interface
sudo cp ./udev/* /etc/udev/rules.d/
```

These rules generate symlinks for the joysticks when plugging them in to 
```/dev/input/js[nameOfJoystick]``` and 
```/dev/input/event[nameOfJoystick]```. 
Currently only tested with one joystick model at a time.

Author(s): Linus Isler

[pluginlib]: http://wiki.ros.org/pluginlib
[sensor_msgs/Joy]: http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html