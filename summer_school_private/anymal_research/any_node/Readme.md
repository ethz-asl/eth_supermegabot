# Any_node (a.k.a. minimal nodewrap)

## Overview
Implements several convenience classes and functions.

### Differences to ROS Nodes
The any_node is a wrapper for the official [ROS node](http://wiki.ros.org/Nodes).
It adds workers (high-precision version of ros::Rate class), custom signal handling and verbose ROS parameter reading, at the cost of less stable API.
If these features are not explicitly required, it is recommended to use the official ROS node.

### Param.hpp
Forwards to param_io package, which allows to read and write ROS messages from and to the parameter server. Also, it provides param(..) functions which print a warning if a requested parameter was not found.

### Topic.hpp
Allows to advertise/subscribe to/from topics/services, whose connection details (topic name, latched, queue_size, deactivate, ...) are saved as ros parameters. Example yaml file:

    publishers:
      my_publisher_name:
        topic: /my_publisher_topic_name
        queue_size: 1
        latch: false

    subscribers:
      my_subscriber_name:
        topic: /my_subscriber_topic_name
        queue_size: 1

    servers:
      my_service_server_name:
        service: my_service_name

    clients:
      my_service_client_name:
        service: my_service_name
        persistent: false


### Node.hpp
Provides an interface base class any_node::Node, which declares init, cleanup and update functions and has a any_worker::WorkerManager instance.
Classes derived from this are compatible with the Nodewrap template.
Additionally, it forwards calls of subscribe, advertise, param, advertiseService serviceClient and addWorker calls to the above mentioned functions.
See any_node_example for an example.

### Nodewrap.hpp
Convencience template, designed to be used with classes derived from any_node::Node.
It automatically sets up ros nodehandlers (with private namespace) and spinners, signal handlers (like SIGINT, ...) and calls the init function on startup and cleanup on shutdown of the given Node.
See any_node_example for an example.

