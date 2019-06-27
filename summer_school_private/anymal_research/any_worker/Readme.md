# Any worker

## Overview

This package provides a Worker and WorkerManager classes. Each Worker owns a thread, which calls a given callback function at given rate. Workers can be added/started and stopped any time.
The user is responsible that the callback functions do not block (otherwise, the worker may not be able to terminate, even if requested to do so).

### Differences to ROS classes

* The ANYbotics `any_worker::Rate` is the equivalent to `ros::Duration`, with a minimal resolution of 1ns instead of 1ms.
* The ANYbotics `any_worker::Worker` is the equivalent to `ros::Timer`, with a minimal resolution of 1ns instead of 1ms. The ANYbotics Worker creates a separate thread instead of running as part of your ROS spinner(s). As it requires thread-safety, only use it if the `ros::Timer` is not accurate enough.
