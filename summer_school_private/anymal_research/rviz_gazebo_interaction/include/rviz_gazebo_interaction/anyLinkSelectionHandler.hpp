/*!
* @file     anyLinkSelectionHandler.hpp
* @author   Linus Isler
* @date     October, 2016
*/

#pragma once

#include <rviz/robot/robot_link.h>
#include "rviz/selection/selection_handler.h"
#include "rviz/properties/property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/vector_property.h"
#include <ros/ros.h>


namespace rviz
{

// RobotLinkSelectionHandler is declared & defined in rviz/robot/robot_link.cpp.
// To avoid the retinterpret- and allow dynamic_casting to AnyLinkSelectionHandler
// the class is rewritten here as a complete Type.

class RobotLinkSelectionHandler : public SelectionHandler
{
protected:
  RobotLink* link_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;

public:
  RobotLinkSelectionHandler( RobotLink* link, DisplayContext* context )
    : SelectionHandler(context)
    , link_(link)
  {}

  virtual ~RobotLinkSelectionHandler(){}
};

} // namespace rviz



namespace rviz_gazebo_interaction {

class AnyLinkSelectionHandler : public rviz::RobotLinkSelectionHandler
{
public:
  AnyLinkSelectionHandler(rviz::RobotLink* link,
                             rviz::DisplayContext* context);
  virtual ~AnyLinkSelectionHandler();

  bool isRobotLink();
  const std::string& getLinkName() const;
  void printLinkName();
  const Ogre::Vector3 getLinkPosition() const;

};

} // namespace rviz_gazebo_interaction
