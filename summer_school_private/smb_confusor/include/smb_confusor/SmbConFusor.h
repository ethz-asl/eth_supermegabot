//
// Created by tim on 16.11.18.
//

#ifndef SMB_SMBCONFUSOR_H
#define SMB_SMBCONFUSOR_H

//#define SYNC_DEBUG
//#define COST_DEBUG

#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>



#include "smb_confusor/SensorEnumDefinition.h"
#include "smb_confusor/SmbState.h"
#include "smb_confusor/BmmPropagator.h"
#include <confusion/modules/external_reference_tracking/ExternalReferenceTrackingModule.h>
#include "confusion/modules/apriltag/AprilTagModule.h"

#include <std_msgs/Empty.h>
//#include <visualization_msgs/MarkerArray.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PoseArray.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include "apriltag/april_tag_detector.hpp"
//#include <confusion/TagArray.h>
#include <tf/transform_broadcaster.h>

#include <confusion/ConFusor.h>
#include <confusion/BatchFusor.h>
#include <confusion/Logger.h>
#include <confusion/utilities/utilities.h>
#include "confusion/utilities/ros_conversions.h"
#include "confusion/utilities/ImuPropagator.h"
#include "confusion/Diagram.h"

class SmbConFusor {
 public:
  SmbConFusor(ros::NodeHandle &node);

 protected:
  void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
  void baseOdometryCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

  void triggerBatchCalCallback(const std_msgs::Empty &msg);
  void drawDiagramCallback(const std_msgs::Empty &msg);

  void runEstimator();

  void stopTracking(const std_msgs::Empty &msg);
  void startTracking(const std_msgs::Empty &msg);
  void stopTracking();
  void startTracking();

  void publish(std::shared_ptr<confusion::State> statePtr, const confusion::StateVector *stateVector);

  ros::NodeHandle &node_;
  ros::Subscriber subImu_;
  ros::Subscriber baseOdometrySub_;
  ros::Subscriber subTriggerBatch_;
  ros::Subscriber subDrawDiagram_;

  image_transport::Publisher imagePub_;
  ros::Publisher pubState_;
  ros::Publisher pubStates_;
  ros::Publisher pubPose_;
  ros::Publisher pubRtPose_;
  ros::Publisher pubLidarPose_;
  ros::Publisher pubRawImuPose_;
  tf::TransformBroadcaster tfBroadcaster_;


  std::thread estimatorThread_;
  confusion::ConFusor conFusor_;

  // Sensor enable flags
  bool useImu_ = true; // todo Should always be true!
  bool useBmm_ = true;
  bool useTags_ = true;
  bool useLidar_ = true;

  int stateInitializationSensor_ = IMU;
  int forwardPropagationSensor_ = IMU;

  // Stuff for base motion model
  BaseMotionModelCalibration baseMotionModelCalibration_;
  confusion::Pose<double> T_imu_base_;
  //todo For development
  BaseMotionModel baseMotionModel_;
  bool bmmMeasReceived_ = false;
  double tBmmStart_;
  bool updateBmmStartingState_ = true;
  confusion::Pose<double> T_w_b_bmm_start_;
  ros::Publisher bmmPub_;
  smb_confusor::BmmPropagator bmmPropagator_;


  // For external reference tracking
  std::unique_ptr<confusion::ExternalReferenceTrackingModule> ertModule_;

  // For tag tracking
  std::unique_ptr<confusion::AprilTagModule> aprilTagInterface_;

  // Stuff for IMU measurements
  Eigen::Vector2d gravity_rot_;
  confusion::ImuCalibration imuCalibration_;

  std::string configFile = "/config/smb_calibration.cfg";
  std::string confusionPath_;
  int batchSize_;
  bool newUpdateMeasReady_ = false;
  bool logData_;
  bool forwardPropagateState_;
  bool tracking_ = false;
  double t_imu_latest_ = 0.0;
  double t_bmm_latest_ = 0.0;
  double t_latest_state_ = 0.0;
  double loop_freq = 100; // [Hz]
  std::unique_ptr<confusion::Logger> logger_;
  boost::property_tree::ptree pt;

  //For saving states and running a batch problem at the end
  confusion::StateVector statesBatch_;
  bool runBatch_ = false;
  bool run_ = true;
  bool runEstimatorLoopDone_ = false;
  int stateToDropIndex_ = -1; //-1 means that we shouldn't drop a state
  bool drawDiagramRequest_ = false;

  confusion::ImuPropagator imuPropagator_;
};

#endif //SMB_SMBCONFUSOR_H
