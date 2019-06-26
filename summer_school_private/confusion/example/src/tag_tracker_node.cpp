//
// Created by tim on 10.10.18.
//

#include "confusion/SensorEnumDefinition.h"
#include "confusion/ImuState.h"
#include "confusion/TagTracker.h"


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "TagTracker");
  ros::NodeHandle nh;

  confusion::TagTracker<ImuState> tagTracker(nh);

  ros::spin();

  return 0;
}
