/*!
 * @file    smb_state_estimator_node.cpp
 * @author  Tim Sandy
 * @date    Oct, 2018
 */

#include <any_node/any_node.hpp>
#include <smb_state_estimator/SmbStateEstimator.h>

int main(int argc, char** argv) {
  any_node::Nodewrap<smb_state_estimator::SmbStateEstimator> node(argc, argv, "smb_state_estimator", 2);
  node.execute();
  return 0;
}
