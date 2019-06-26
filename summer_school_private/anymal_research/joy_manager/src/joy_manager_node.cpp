/*!
* @file     joy_manager_node.cpp
* @author   Linus Isler
* @date     May, 2016
* @brief
*/

#include <ros/ros.h>
#include <any_node/any_node.hpp>

#include "joy_manager/JoyManager.hpp"



int main(int argc, char **argv)
{
  any_node::Nodewrap<joy_manager::JoyManager> node(argc, argv, "joy_manager", 1);
  node.execute(); 
  // execute blocks until the node was requested to shut down (after reception of a signal (e.g. SIGINT) or after calling the any_node::Node::shutdown() function)
  return 0;
}
