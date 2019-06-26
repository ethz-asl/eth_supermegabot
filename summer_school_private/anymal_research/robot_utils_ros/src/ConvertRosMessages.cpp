
// series elastic actuator ros
#include "robot_utils_ros/force_calibrators/ConvertRosMessages.hpp"

namespace robot_utils_ros {

void ConvertRosMessages::writeToMessage(robot_utils_ros::ForceCalibratorCommand& message, const robot_utils::ForceCalibratorCommand& command) {
      message.cmd_start = command.cmdStart_;
      message.cmd_continue = command.cmdContinue_;
      message.cmd_calibrate = command.cmdCalibrate_;
      message.enable_outlier_detector = command.enableOutlierDetector_;
      message.num_samples = command.numSamples_;
      message.num_good_samples = command.numGoodSamples_;
}



void ConvertRosMessages::readFromMessage(robot_utils::ForceCalibratorCommand& command, const robot_utils_ros::ForceCalibratorCommand& message) {
      command.cmdStart_ = message.cmd_start;
      command.cmdContinue_ = message.cmd_continue;
      command.cmdCalibrate_ = message.cmd_calibrate;
      command.enableOutlierDetector_ = message.enable_outlier_detector;
      command.numSamples_ = message.num_samples;
      command.numGoodSamples_ = message.num_good_samples;

}


}