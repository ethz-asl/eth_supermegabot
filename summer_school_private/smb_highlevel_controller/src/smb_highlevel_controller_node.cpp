/*
 * smb_highlevel_controller_node.cpp
 *
 *  Created on: Oct, 2018
 *      Author: Tim Sandy
 */

#include <smb_highlevel_controller/SmbHighLevelController.hpp>

// nodewrap
#include <any_node/Nodewrap.hpp>


int main(int argc, char **argv)
{
  any_node::Nodewrap<smb_highlevel_controller::SmbHighLevelController> node(argc, argv, "smb_highlevel_controller");
  node.execute();
  return 0;
}
