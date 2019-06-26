// any node
#include "any_node/any_node.hpp"

// smb lowlevel controller
#include "smb_lowlevel_controller/SmbLowLevelController.hpp"


int main(int argc, char** argv)
{
  any_node::Nodewrap<smb_lowlevel_controller::SmbLowLevelController> node(argc, argv, "smb_lowlevel_controller", 2); // use 2 spinner thread
  node.execute(); // update thread won't be executed
  return 0;
}
