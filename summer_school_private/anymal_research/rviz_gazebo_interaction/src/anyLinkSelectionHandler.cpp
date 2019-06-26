/*!
* @file     anyLinkSelectionHandler.cpp
* @author   Linus Isler
* @date     October, 2016
*/

#include "rviz_gazebo_interaction/anyLinkSelectionHandler.hpp"

namespace rviz_gazebo_interaction {

AnyLinkSelectionHandler::AnyLinkSelectionHandler(rviz::RobotLink* link, rviz::DisplayContext* context)
  : RobotLinkSelectionHandler(link, context)
{

}


AnyLinkSelectionHandler::~AnyLinkSelectionHandler() { }


bool AnyLinkSelectionHandler::isRobotLink() {
  if (link_) {
    return true;
  }
  return false;
}


const std::string& AnyLinkSelectionHandler::getLinkName() const {
  return link_->getName();
}


void AnyLinkSelectionHandler::printLinkName() {
  ROS_INFO_STREAM(link_->getName());
}

const Ogre::Vector3 AnyLinkSelectionHandler::getLinkPosition() const {
  return link_->getPosition();
}

} //namespace rviz_gazebo_interaction

