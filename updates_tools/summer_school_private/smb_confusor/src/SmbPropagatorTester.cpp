//
// Created by tim on 28.01.19.
//

#include "smb_confusor/SmbPropagatorTester.h"

namespace smb_confusor {

SmbPropagatorTester::SmbPropagatorTester(ros::NodeHandle &node) {
  //Load IMU configuration
  std::string confusionPath = ros::package::getPath("confusion");
  std::string configFile = confusionPath + "/config/tagtracker_config.xml";
  boost::property_tree::ptree confusionPt;
  boost::property_tree::read_info(configFile, confusionPt);

  imuCalibration_.gravityMagnitude_ = confusionPt.get<double>("gravityMagnitude");
  auto wi_stddev = confusionPt.get<double>("wi_stddev");
  auto ai_stddev = confusionPt.get<double>("ai_stddev");
  auto bg_stddev = confusionPt.get<double>("bg_stddev");
  auto ba_stddev = confusionPt.get<double>("ba_stddev");
  imuCalibration_.cov_imu_nominal_.setIdentity();
  imuCalibration_.cov_imu_nominal_.block<3, 3>(0, 0) *= wi_stddev * wi_stddev;
  imuCalibration_.cov_imu_nominal_.block<3, 3>(3, 3) *= ai_stddev * ai_stddev;
  imuCalibration_.cov_imu_nominal_.block<3, 3>(6, 6) *= bg_stddev * bg_stddev;
  imuCalibration_.cov_imu_nominal_.block<3, 3>(9, 9) *= ba_stddev * ba_stddev;

  imuCalibration_.g_w_ << 0.0, 0.0, imuCalibration_.gravityMagnitude_;
  gravity_rot_.setZero();

  //Load the BMM configuration
  std::string packagePath = ros::package::getPath("smb_confusor");
  std::string configFile2 = packagePath + "/config/baseMotionModelCalibration.xml";
  boost::property_tree::ptree wsePt;
  boost::property_tree::read_info(configFile2, wsePt);

  T_imu_base_.trans(0) = wsePt.get<double>("T_imu_base.px");
  T_imu_base_.trans(1) = wsePt.get<double>("T_imu_base.py");
  T_imu_base_.trans(2) = wsePt.get<double>("T_imu_base.pz");
  T_imu_base_.rot.w() = wsePt.get<double>("T_imu_base.qw");
  T_imu_base_.rot.x() = wsePt.get<double>("T_imu_base.qx");
  T_imu_base_.rot.y() = wsePt.get<double>("T_imu_base.qy");
  T_imu_base_.rot.z() = wsePt.get<double>("T_imu_base.qz");
  T_imu_base_.rot.normalize();

//  useBmm_ = wsePt.get<bool>("useBmm");
  baseMotionModelCalibration_.useLossFunction_ = wsePt.get<bool>("bmmUseLossFunction");
  baseMotionModelCalibration_.lossCoefficient_ = wsePt.get<double>("bmmLossCoefficient");
  double bmm_cov_stationary_actuated = wsePt.get<double>("bmm_cov_stationary_actuated");
  double bmm_cov_stationary_unactuated = wsePt.get<double>("bmm_cov_stationary_unactuated");
  double bmm_cov_moving_actuated = wsePt.get<double>("bmm_cov_moving_actuated");
  double bmm_cov_moving_unactuated = wsePt.get<double>("bmm_cov_moving_unactuated");

  baseMotionModelCalibration_.cov_Twb_stationary.setIdentity();
  baseMotionModelCalibration_.cov_Twb_stationary *= bmm_cov_stationary_unactuated;
  baseMotionModelCalibration_.cov_Twb_stationary(2,2) = bmm_cov_stationary_actuated;
  baseMotionModelCalibration_.cov_Twb_stationary(3,3) = bmm_cov_stationary_actuated;

  baseMotionModelCalibration_.cov_Twb_moving.setIdentity();
  baseMotionModelCalibration_.cov_Twb_moving *= bmm_cov_moving_unactuated;
  baseMotionModelCalibration_.cov_Twb_moving(2,2) = bmm_cov_moving_actuated;
  baseMotionModelCalibration_.cov_Twb_moving(3,3) = bmm_cov_moving_actuated;

  baseMotionModelCalibration_.movingWheelSpeedThld_ = wsePt.get<double>("movingWheelSpeedThld");
  baseMotionModelCalibration_.wheelRadius_ = wsePt.get<double>("wheelRadius");
  baseMotionModelCalibration_.wheelbase_ = wsePt.get<double>("wheelbase");

  //Set up subscribers and publishers
  subImu_ = node.subscribe("/imu0", 100, &SmbPropagatorTester::imuCallback,
                            this, ros::TransportHints().tcpNoDelay());
  baseOdometrySub_ = node.subscribe("/smb_lowlevel_controller/actuator_readings", 10,
                                    &SmbPropagatorTester::baseOdometryCallback, this);
  stateEstimateSub_ = node.subscribe("/pose_est_out", 10, &SmbPropagatorTester::stateEstimateCallback, this);
}

void SmbPropagatorTester::imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg) {
  double t_imu_source = imuMsg->header.stamp.toSec();

  Eigen::Vector3d a(imuMsg->linear_acceleration.x,
                    imuMsg->linear_acceleration.y,
                    imuMsg->linear_acceleration.z);
  Eigen::Vector3d w(imuMsg->angular_velocity.x,
                    imuMsg->angular_velocity.y,
                    imuMsg->angular_velocity.z);

  smbPropagator_.addProcessMeasurement(std::make_shared<confusion::ImuMeas>(
      t_imu_source, a, w, &imuCalibration_, &gravity_rot_));

#ifdef SYNC_DEBUG
  std::cout << "Imu meas received with stamp " << t_imu_source << std::endl;
#endif
}

void SmbPropagatorTester::baseOdometryCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  auto wheelSpeedMeasurement = std::make_shared<WheelSpeeds>(
      msg->data[0],
      msg->data[1],
      msg->data[2],
      baseMotionModelCalibration_,
//      *T_armbase_imu_ptr_,
      T_imu_base_,
      BMM);

//  if (useBmm_) {
  smbPropagator_.addProcessMeasurement(wheelSpeedMeasurement);
//  }

#ifdef SYNC_DEBUG
  std::cout << "Added wheel speed measurement for t=" << msg->data[0] - t_epoch_ <<
        ". leftWheelSpd=" << msg->data[1] << ", rightWheelSpd=" << msg->data[2] << std::endl;
#endif
}

void SmbPropagatorTester::stateEstimateCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  if (msg->data.size() != 17) {
    std::cout << "[SmbPropagatorTester] State estimate message received with the wrong size" << std::endl;
    return;
  }

  confusion::ImuStateParameters imuStateUpdate;
  imuStateUpdate.t_ = msg->data[0];
  imuStateUpdate.T_w_i_ = confusion::Pose<double>(msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
  imuStateUpdate.T_w_i_.rot.normalize();
  imuStateUpdate.linVel_ << msg->data[8],msg->data[9],msg->data[10];
  imuStateUpdate.accelBias_ << msg->data[11],msg->data[12],msg->data[13];
  imuStateUpdate.gyroBias_ << msg->data[14],msg->data[15],msg->data[16];

  smbPropagator_.setEstimate(imuStateUpdate);

#ifdef IMU_PROP_DEBUG
  MELO_INFO("Update received for t=%10.10f\n", imuStateUpdate.t_);
#endif

//  //Do some sanity checks
//  if (imuPropagator_.isBufferEmpty())
//    MELO_WARN("[SmbStateEstimator] imuPropagator is empty when applying a state update!?");
//
//  if (imuStateUpdate.t_ < imuPropagator_.getFrontMeasurement().t() || imuStateUpdate.t_ > imuPropagator_.getBackMeasurement().t())
//    MELO_WARN("[SmbStateEstimator] Time mismatch in state update! t_state=%10.10f, t_imu_back=%10.10f, t_imu_front=%10.10f\n",
//              imuStateUpdate.t_, imuPropagator_.getBackMeasurement().t(), imuPropagator_.getFrontMeasurement().t());
//
//  imuPropagator_.update(imuStateUpdate);
//
//  stateEstimateReceived_ = true;
}

} // namespace smb_confusor


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "SmbPropagatorTester");
  ros::NodeHandle nh;

  smb_confusor::SmbPropagatorTester wpt(nh);

  ros::Rate r(250);
  confusion::ImuStateParameters stateParams;
  tf::TransformBroadcaster tfBroadcaster_;
  while (ros::ok()) {
    if (wpt.smbPropagator_.getPropagatedEstimate(ros::Time::now().toSec(), stateParams)) {
      //Publish tf message
      tfBroadcaster_.sendTransform(tf::StampedTransform(confusion::getTfMsg(stateParams.T_w_i_),
                                                        ros::Time::now(), "world", "imu"));
    }

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}

