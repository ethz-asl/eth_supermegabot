/*!
* @file     feedback_node.cpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/

#include <ros/ros.h>
#include <any_node/any_node.hpp>

#include "joy_interface/Feedback.hpp"



int main(int argc, char **argv)
{
  int numSpinners = 1;
  any_node::Nodewrap<joy_interface::Feedback> node(argc, argv, "feedback", numSpinners);
  node.execute(); 
  // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the any_node::Node::shutdown() function)
  return 0;
}
