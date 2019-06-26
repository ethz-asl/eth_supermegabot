/*!
* @file     Feedback.cpp
* @author   Linus Isler
* @date     June 21th, 2016
* @brief
*/


#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <sys/ioctl.h>

#include <sensor_msgs/JoyFeedback.h>
#include "joy_interface/Feedback.hpp"


namespace joy_interface {

Feedback::Feedback(any_node::Node::NodeHandlePtr nh)
: any_node::Node(nh),
  fileDescriptor_(0),
  initializedJoystick_(false),
  isFirstFaultOpening_(true)
{}

bool Feedback::init() {
  ROS_DEBUG_STREAM("Feedback feedback init()");
  deviceName_ = param<std::string>("feedback_dev", "/dev/input/event15");

  joystickFeedbackSubscriber_ = subscribe("feedback",
                                          "/feedback/operator",
                                          10,
                                          &Feedback::joystickFeedbackCallback,
                                          this);

  return addWorker(ros::this_node::getName() + "::updateWorker", param<double>("time_step", 1.0), &Feedback::update, this, 90);
}

bool Feedback::update(const any_worker::WorkerEvent& event) {
  if(!initializedJoystick_) {
    if (isFirstFaultOpening_) {
      ROS_WARN_STREAM("Could'nt open joystick feedback device " << 
        deviceName_ << ". Will retry every second.\n Error: " << strerror(errno));
      isFirstFaultOpening_ = false;
    }

    fileDescriptor_ = open(deviceName_.c_str(), O_RDWR);
    if (fileDescriptor_ == -1) {
      return true;
    }

    double gainDouble = param<double>("feedback_gain", 0xFFFF);
    struct input_event gain;
    gain.type = EV_FF;
    gain.code = FF_GAIN;
    gain.value = gainDouble;

    if (write(fileDescriptor_, &gain, sizeof(gain)) != sizeof(gain)) {
      ROS_ERROR_STREAM("Couldn't set gain for Logitech F710 feedback: " << strerror(errno));
      ros::shutdown();
    }

    int max_effects;
    if (ioctl(fileDescriptor_, EVIOCGEFFECTS, &max_effects) < 0 || max_effects < 1) {
      ROS_ERROR_STREAM("ioctl max num effects query " << strerror(errno));
      ros::shutdown();
    }

    effects_.resize(max_effects);

    const int index = 0;
    effects_[index].type = FF_RUMBLE;
    effects_[index].id = -1;
    effects_[index].u.rumble.strong_magnitude = 0xffff; // [0, 0xFFFF]
    effects_[index].u.rumble.weak_magnitude   = 0xffff; // [0, 0xFFFF]
    effects_[index].replay.length = 500;  // 0.5 seconds
    effects_[index].replay.delay = 0;

    if (ioctl(fileDescriptor_, EVIOCSFF, &effects_[index]) < 0) {
      ROS_ERROR_STREAM("ioctl upload effect " << index << " " << strerror(errno));
      ros::shutdown();
    }

    initializedJoystick_ = true;
  }

  return true;
}

void Feedback::joystickFeedbackCallback(const sensor_msgs::JoyFeedbackArray::ConstPtr& feedbackArray) {
  for (sensor_msgs::JoyFeedback feedback : feedbackArray->array) {
    if (feedback.type == feedback.TYPE_RUMBLE && effects_.size() > feedback.id) {
      effects_[feedback.id].u.rumble.strong_magnitude = feedback.intensity * 0xffff;
      effects_[feedback.id].u.rumble.weak_magnitude = feedback.intensity * 0xffff;
      if (ioctl(fileDescriptor_, EVIOCSFF, &effects_[feedback.id]) < 0)
        ROS_WARN_STREAM("ioctl upload effect " << feedback.id << " " << strerror(errno));
      struct input_event play;
      play.type = EV_FF;
      play.code = effects_[feedback.id+1].id;
      if (play.code < 0) return;
      play.value = 1;
      if (write(fileDescriptor_, (const void*) &play, sizeof(play)) == -1) {
        ROS_WARN_STREAM("Error while playing feedback: " << strerror(errno));
      }
    }
  }

}



} // namespace joy_interface
