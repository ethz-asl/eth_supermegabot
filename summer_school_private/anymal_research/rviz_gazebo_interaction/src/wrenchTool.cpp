/*!
* @file     wrenchTool.cpp
* @author   Linus Isler
* @date     October, 2016
*/

#include "rviz_gazebo_interaction/wrenchTool.hpp"

#include <OGRE/OgreCamera.h>

#include <rviz/display_context.h>
#include <rviz/render_panel.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include "rviz/selection/selection_handler.h"
#include "rviz/selection/selection_manager.h"
#include <rviz/viewport_mouse_event.h>
#include <rviz/view_manager.h>


namespace rviz_gazebo_interaction
{

WrenchTool::WrenchTool()
  : currentForceVectorProperty_(NULL)
  , currentForceValueProperty_(NULL)
  , currentRobotLink_(NULL)
  , focalDistanceProperty_(NULL)
  , newtonPerMeterProperty_(NULL)
{
  shortcut_key_ = 'w';
}


WrenchTool::~WrenchTool() {

}


void WrenchTool::onInitialize() {
  moveTool_.initialize( context_ );
  lastSelectionFrameCount_ = context_->getFrameCount();
  gazeboWrenchClient_ = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  markerPub_ = nh_.advertise<visualization_msgs::Marker>("wrench_marker", 1);
  deactivate();
}


void WrenchTool::activate() {
  context_->getSelectionManager()->setTextureSize(1);
  arrowMarker_.header.frame_id = context_->getFixedFrame().toStdString();
  arrowMarker_.type = visualization_msgs::Marker::ARROW;
  arrowMarker_.header.stamp = ros::Time::now();
  arrowMarker_.ns = "arrow";
  arrowMarker_.id = 0;
  arrowMarker_.color.r = 0.0f;
  arrowMarker_.color.g = 1.0f;
  arrowMarker_.color.b = 0.0f;
  arrowMarker_.color.a = 1.0;
  arrowMarker_.lifetime = ros::Duration();
  currentForceVectorProperty_ = new rviz::VectorProperty("Force vector");
  currentForceVectorProperty_->setReadOnly(true);
  currentForceValueProperty_ = new rviz::FloatProperty("Force value [N]");
  currentForceValueProperty_->setReadOnly(true);
  focalDistanceProperty_ = new rviz::FloatProperty("Focal Distance [Pixel]");
  focalDistanceProperty_->setFloat(kDefaultFocalDistance);
  newtonPerMeterProperty_ = new rviz::FloatProperty("Newtons per meter [N/m]");
  newtonPerMeterProperty_->setFloat(kDefaultNewtonPerMeter);
  getPropertyContainer()->addChild(currentForceVectorProperty_);
  getPropertyContainer()->addChild(currentForceValueProperty_);
  getPropertyContainer()->addChild(focalDistanceProperty_);
  getPropertyContainer()->addChild(newtonPerMeterProperty_);
}


void WrenchTool::deactivate() {
  delete currentForceVectorProperty_;
  currentForceVectorProperty_ = NULL;
  delete currentForceValueProperty_;
  currentForceValueProperty_ = NULL;
  delete focalDistanceProperty_;
  focalDistanceProperty_ = NULL;
  delete newtonPerMeterProperty_;
  newtonPerMeterProperty_ = NULL;
}


int WrenchTool::processMouseEvent( rviz::ViewportMouseEvent& event ) {
  int flags = 0;
  if (event.panel->contextMenuVisible()) {
    return flags;
  }
  // make sure we let the vis. manager render at least one frame between selection updates
  bool needSelectionUpdate =
      context_->getFrameCount() > lastSelectionFrameCount_;

  // We are dragging if a button was down and is still down
  Qt::MouseButtons buttons = event.buttons_down & (Qt::LeftButton
                                                   | Qt::RightButton 
                                                   | Qt::MidButton);
  if (event.type == QEvent::MouseButtonPress)
    buttons &= ~event.acting_button;
  bool dragging = buttons != 0;

  // unless we're dragging, check if there's a new object under the mouse
  if( needSelectionUpdate &&
      !dragging &&
      event.type != QEvent::MouseButtonRelease ) {
    updateRobotLink( event );
    flags = Render;
  }
  if (currentRobotLink_ && event.leftDown()) {
    setStartPosition(event);
  }
  if (currentRobotLink_ && event.left()) {
    applyWrench(event);
  }
  else if(!currentRobotLink_ && event.panel->getViewController()) {
    moveTool_.processMouseEvent( event );
    setCursor( moveTool_.getCursor() );
  }
  if(event.type == QEvent::MouseButtonRelease) {
    sendWrenchToGazebo(Ogre::Vector3(0.0, 0.0, 0.0));
    arrowMarker_.action = visualization_msgs::Marker::DELETE;
    markerPub_.publish(arrowMarker_);
    updateRobotLink(event);
  }
  return flags;
}


void WrenchTool::updateRobotLink( const rviz::ViewportMouseEvent& event ) {
  rviz::M_Picked results;
  // Pick exactly 1 pixel
  context_->getSelectionManager()->pick( event.viewport,
                                         event.x, event.y,
                                         event.x + 1, event.y + 1,
                                         results, true );

  lastSelectionFrameCount_ = context_->getFrameCount();

  // look for a valid handle in the result.
  rviz::M_Picked::iterator result_it = results.begin();
  if(result_it != results.end()) {
    rviz::Picked pick = result_it->second;
    rviz::SelectionHandler* handler =
        context_->getSelectionManager()->getHandler( pick.handle );
    if (pick.pixel_count > 0 && handler) {
      rviz::RobotLinkSelectionHandler* rHandler =
          dynamic_cast<rviz::RobotLinkSelectionHandler*>(handler);
      if (rHandler) {
        rviz_gazebo_interaction::AnyLinkSelectionHandler* aHandler =
            static_cast<rviz_gazebo_interaction::AnyLinkSelectionHandler*>(rHandler);
        if (aHandler && aHandler->isRobotLink()) {
          ROS_DEBUG_STREAM(aHandler->getLinkName());
          currentRobotLink_ = aHandler;
        }
      }
      else {
       currentRobotLink_ = NULL;
      }
    }
  }
  else {
    currentRobotLink_ = NULL;
  }
}


void WrenchTool::applyWrench(const rviz::ViewportMouseEvent& event ) {
  Ogre::Vector2 endScreenPosition(event.x, event.y);
  ROS_DEBUG_STREAM("screen position("<< endScreenPosition.x << ",\t" << endScreenPosition.y <<")");
  Ogre::Vector3 deltaScreenVector(endScreenPosition.x - startScreenPosition_.x,
                                  startScreenPosition_.y - endScreenPosition.y,
                                  0.0);
  ROS_DEBUG_STREAM("delta screen vector = (" << deltaScreenVector.x << ",\t" << deltaScreenVector.y << ",\t" << deltaScreenVector.z << ")");
  Ogre::Vector3 deltaWorldVector = cameraWorldOrientation_
                                   * deltaScreenVector
                                   * distanceCameraToObject_
                                   / focalDistanceProperty_->getFloat();
  ROS_DEBUG_STREAM("delta world vector = (" << deltaWorldVector.x << ",\t" << deltaWorldVector.y << ",\t" << deltaWorldVector.z << ")");
  publishArrow(Ogre::Vector3(1.0, 0.0, 0.0).getRotationTo(deltaWorldVector),
               deltaWorldVector.length());

  Ogre::Vector3 forceVector = deltaWorldVector
                              * newtonPerMeterProperty_->getFloat();
  sendWrenchToGazebo(forceVector);
  currentForceValueProperty_->setFloat(forceVector.length());
  currentForceVectorProperty_->setVector(forceVector);
}


bool WrenchTool::setStartPosition(const rviz::ViewportMouseEvent& event) {
  bool success = context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, startWorldPosition_);
  if (success) {
    ROS_DEBUG_STREAM(currentRobotLink_->getLinkName());
    startScreenPosition_ = Ogre::Vector2(event.x,event.y);
    ROS_DEBUG_STREAM("screen position = (" << startScreenPosition_.x << ",\t" << startScreenPosition_.y << ")");
    cameraWorldPosition_ = context_->getViewManager()->getCurrent()->getCamera()->getPosition();
    ROS_DEBUG_STREAM("camera position = (" << cameraWorldPosition_.x << ",\t" << cameraWorldPosition_.y << ",\t" << cameraWorldPosition_.z << ")");
    cameraWorldOrientation_ = context_->getViewManager()->getCurrent()->getCamera()->getOrientation();
    ROS_DEBUG_STREAM("camera orientation = (" << cameraWorldOrientation_.getPitch() << ",\t" << cameraWorldOrientation_.getRoll() << ",\t" << cameraWorldOrientation_.getYaw() << ")");
    distanceCameraToObject_ = (startWorldPosition_ - cameraWorldPosition_).length();
    ROS_DEBUG_STREAM("distance = " << distanceCameraToObject_);
  }
  return success;
}


void WrenchTool::publishArrow(const Ogre::Quaternion& orientation, double scale) {
  arrowMarker_.action = visualization_msgs::Marker::ADD;
  arrowMarker_.pose.position.x = startWorldPosition_.x;
  arrowMarker_.pose.position.y = startWorldPosition_.y;
  arrowMarker_.pose.position.z = startWorldPosition_.z;
  arrowMarker_.pose.orientation.x = orientation.x;
  arrowMarker_.pose.orientation.y = orientation.y;
  arrowMarker_.pose.orientation.z = orientation.z;
  arrowMarker_.pose.orientation.w = orientation.w;

  arrowMarker_.scale.x = scale;
  arrowMarker_.scale.y = 0.1 * scale;
  arrowMarker_.scale.z = 0.1 * scale;
  markerPub_.publish(arrowMarker_);
}


bool WrenchTool::sendWrenchToGazebo(const Ogre::Vector3& force) {
  gazebo_msgs::ApplyBodyWrench srv;
  if (!currentRobotLink_) {
    return false;
  }
  srv.request.body_name = currentRobotLink_->getLinkName();
  srv.request.reference_point.x = startWorldPosition_.x - currentRobotLink_->getLinkPosition().x;
  srv.request.reference_point.y = startWorldPosition_.y - currentRobotLink_->getLinkPosition().y;
  srv.request.reference_point.z = startWorldPosition_.z - currentRobotLink_->getLinkPosition().z;
  srv.request.wrench.force.x = force.x;
  srv.request.wrench.force.y = force.y;
  srv.request.wrench.force.z = force.z;
  srv.request.duration = ros::Duration(-1);
  if (gazeboWrenchClient_.call(srv)) {
    return true;
  }
  return false;
}


void WrenchTool::save( rviz::Config config ) const {

}

void WrenchTool::load( const rviz::Config& config ) {

}

} // namespace rviz_gazebo_interaction

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_gazebo_interaction::WrenchTool,rviz::Tool )
