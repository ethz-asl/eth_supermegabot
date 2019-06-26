#include "rocoma_rqt/WorkerThreadSetMode.h"

void WorkerThreadSetMode::run() {
  bool isOk = switchControllerModeClient_.call(request_, response_);
  emit resultReady(isOk, response_);
}

void WorkerThreadSetMode::setRequest(rocoma_msgs::SwitchControllerRequest request) { request_ = request; }

void WorkerThreadSetMode::setClient(ros::ServiceClient& client) { switchControllerModeClient_ = client; }
