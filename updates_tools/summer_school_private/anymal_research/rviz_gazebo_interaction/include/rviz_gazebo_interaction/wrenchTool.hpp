/*!
* @file     wrenchTool.hpp
* @author   Linus Isler
* @date     October, 2016
*/

#pragma once

#include <gazebo_msgs/ApplyBodyWrench.h>
#include <rviz/default_plugin/tools/move_tool.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/tool.h>
#include <rviz_gazebo_interaction/anyLinkSelectionHandler.hpp>
#include <visualization_msgs/Marker.h>


namespace rviz_gazebo_interaction
{

// Approximately and identified experimentally.
// Can be set by the user and confirmed with the marker.
static constexpr double kDefaultFocalDistance = 940.0;

// Reasonable value that can be changed by the user.
static constexpr double kDefaultNewtonPerMeter = 100.0;


class WrenchTool: public rviz::Tool
{
Q_OBJECT
public:
  WrenchTool();
  ~WrenchTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

public:
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

private:
  void updateRobotLink(const rviz::ViewportMouseEvent& event);
  void applyWrench(const rviz::ViewportMouseEvent& event);
  bool setStartPosition(const rviz::ViewportMouseEvent& event);
  void publishArrow(const Ogre::Quaternion& orientation, double scale);
  bool sendWrenchToGazebo(const Ogre::Vector3& force);

  ros::NodeHandle nh_;
  ros::ServiceClient gazeboWrenchClient_;
  ros::Publisher markerPub_;
  uint64_t lastSelectionFrameCount_;
  rviz::MoveTool moveTool_;

  rviz_gazebo_interaction::AnyLinkSelectionHandler* currentRobotLink_;
  Ogre::Vector2 startScreenPosition_;
  Ogre::Vector3 startWorldPosition_;
  Ogre::Vector3 cameraWorldPosition_;
  Ogre::Quaternion cameraWorldOrientation_;
  double distanceCameraToObject_;

  visualization_msgs::Marker arrowMarker_;
  rviz::VectorProperty* currentForceVectorProperty_;
  rviz::FloatProperty* currentForceValueProperty_;
  rviz::FloatProperty* focalDistanceProperty_;
  rviz::FloatProperty* newtonPerMeterProperty_;
};

} // namespace rviz_gazebo_interaction
