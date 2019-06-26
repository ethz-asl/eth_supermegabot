// signal logger example
#include "signal_logger_example/SignalLoggerExample.hpp"

// ros
#include "ros/ros.h"

// nodewrap
#include "any_node/Nodewrap.hpp"

int main(int argc, char **argv)
{
  any_node::Nodewrap<signal_logger_example::SignalLoggerExample> node(argc, argv, "signal_logger_example", 4);
  node.execute();
  return 0;
}
