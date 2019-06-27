# param_io

## Overview

Read and write ROS messages from and to the parameter server.
Wraps the standard ROS parameter interface, adding verbose warning messages if a parameter is not found.

This code does not support all message types and can be extended when needed.

Example for a TwistStamped message in yaml syntax.

    header:
      stamp:
        sec:                         0
        nsec:                        0
      seq:                           0
      frame_id:                      base
    twist:
      linear:
        x:                           1.0
        y:                           2.0
        z:                           3.0
      angular:
        x:                           4.0
        y:                           5.0
        z:                           6.0
