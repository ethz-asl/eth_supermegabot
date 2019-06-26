# yaml_tools

## Overview

Edit yaml nodes, read and write yaml files.

Wraps around the yaml-cpp library.

Advantages:

* Provides a simpler interface.
* Catches all yaml-cpp exceptions, throws exceptions of type yaml_tools::Exception with more verbose warning and error statements.
* Is a catkin package.
* Fixed issues which yaml-cpp has, e.g. integer doubles are stored as int, infinity and NaN are not treated correctly.
* Maps are processed in alphabetic order.

Current limitations:

* Maps can only have strings as keys.
