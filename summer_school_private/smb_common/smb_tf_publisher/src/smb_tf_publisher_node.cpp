/*!
* @file     smb_tf_publisher_node.cpp
* @author   Johannes Pankert
* @date     June 9, 2019
* @brief
*/

#include <ros/ros.h>
#include <any_node/any_node.hpp>
#include "smb_tf_publisher/SmbTfPublisher.hpp"


int main(int argc, char **argv)
{
  any_node::Nodewrap<smb_tf_publisher::SmbTfPublisher> node(argc, argv, "smb_tf_publisher", 1);
  node.execute();
  return 0;
}


