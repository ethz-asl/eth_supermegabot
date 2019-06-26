/*!
 * @file     SmbDescription.hpp
 * @author   Koen Kraemer
 * @date     Aug 1, 2018
 */

#pragma once

// smb description
#include <smb_description/topology.hpp>
#include <smb_description/definitions.hpp>
//#include <smb_description/Containers.hpp>
#include <smb_description/robot_description_additions.hpp>

namespace smb_description {

using ConcreteSmbDescription = romo::ConcreteDescription<SmbDefinitions, SmbTopology>;
using SmbDescription = romo::RobotDescription<ConcreteSmbDescription>;
//using SmbContainers = romo::RobotContainers<SmbContainersImpl>;

} /* namespace smb_description */