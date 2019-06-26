/*!
 * @file  cosmo_node.hpp
 * @author  Philipp Leemann
 * @date  May, 2017
 */

#pragma once

#include "cosmo_node/Node.hpp"

#include "any_node/Nodewrap.hpp"

namespace cosmo_node {

template <class NodeImpl>
using Nodewrap = any_node::Nodewrap<NodeImpl>;

}