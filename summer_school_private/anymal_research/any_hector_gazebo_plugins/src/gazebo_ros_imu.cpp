//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <any_hector_gazebo_plugins/gazebo_ros_imu.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/console.h>

#include <kindr/Core>

namespace gazebo
{

// #define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
  #include <geometry_msgs/PoseStamped.h>
  static ros::Publisher debugPublisher;
#endif // DEBUG_OUTPUT

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_accel_.reset();
  dynamic_reconfigure_server_rate_.reset();
  dynamic_reconfigure_server_yaw_.reset();

  node_handle_->shutdown();
#ifdef USE_CBQ
  callback_queue_thread_.join();
#endif
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  world = _model->GetWorld();

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  else
    namespace_.clear();

  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }
  else
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }

  // assert that the body by link_name_ exists
  if (!link)
  {
    ROS_FATAL("GazeboRosIMU plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = link_name_;
  topic_ = "imu";

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("biasTopicName"))
    bias_topic_ = _sdf->GetElement("biasTopicName")->GetValue()->GetAsString();
  else
    bias_topic_ = (!topic_.empty() ? topic_ + "/bias" : "");

  if (_sdf->HasElement("serviceName"))
    serviceName = _sdf->GetElement("serviceName")->GetValue()->GetAsString();
  else
    serviceName = topic_ + "/calibrate";

  accelModel.Load(_sdf, "accel");
  rateModel.Load(_sdf, "rate");
  yawModel.Load(_sdf, "yaw");

  // also use old configuration variables from gazebo_ros_imu
  if (_sdf->HasElement("gaussianNoise")) {
    double gaussianNoise;
    if (_sdf->GetElement("gaussianNoise")->GetValue()->Get(gaussianNoise) && gaussianNoise != 0.0) {
      accelModel.gaussian_noise = gaussianNoise;
      rateModel.gaussian_noise  = gaussianNoise;
    }
  }

  if (_sdf->HasElement("xyzOffset")) {
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d xyzOffset = _sdf->Get<ignition::math::Vector3d>("xyzOffset");
    T_BM_.getPosition() = -kindr::Position3D(xyzOffset.X(), xyzOffset.Y(), xyzOffset.Z());
#else
    math::Vector3 xyzOffset = _sdf->Get<math::Vector3>("xyzOffset");
    T_BM_.getPosition() = -kindr::Position3D(xyzOffset.x, xyzOffset.y, xyzOffset.z);
#endif
  } else {
    ROS_INFO("imu plugin missing <xyzOffset>, defaults to 0s");
    T_BM_.getPosition().setZero();
  }

  if (_sdf->HasElement("rpyOffset")) {
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Quaterniond q = _sdf->Get<ignition::math::Quaterniond>("rpyOffset");
    T_BM_.getRotation() = kindr::RotationQuaternionD(q.W(),q.X(),q.Y(),q.Z()).inverted();
#else
    math::Quaternion q = _sdf->Get<math::Vector3>("rpyOffset");
    T_BM_.getRotation() = kindr::RotationQuaternionD(q.w,q.x,q.y,q.z).inverted();
#endif
  } else {
    ROS_INFO("imu plugin missing <rpyOffset>, defaults to 0s");
    T_BM_.getRotation().setIdentity();
  }

  rosImuMsg.header.frame_id = frame_id_;
  biasMsg.header.frame_id = frame_id_;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // if topic name specified as empty, do not publish (then what is this plugin good for?)
  if (!topic_.empty()) {
    cosmo_ros::PublisherRosOptionsPtr pub_options = std::make_shared<cosmo_ros::PublisherRosOptions>("/" + namespace_ + "/" + topic_, *node_handle_);
    pub_ = cosmo_ros::advertiseShmRos<any_measurements::Imu, sensor_msgs::Imu, any_measurements_ros::ConversionTraits>(pub_options);
  }
  if (!bias_topic_.empty()) {
    bias_pub_ = node_handle_->advertise<sensor_msgs::Imu>(bias_topic_, 10);
  }

#ifdef DEBUG_OUTPUT
  debugPublisher = rosnode_->advertise<geometry_msgs::PoseStamped>(topic_ + "/pose", 10);
#endif // DEBUG_OUTPUT

  // advertise services for calibration and bias setting
  if (!serviceName.empty())
    srv_ = node_handle_->advertiseService(serviceName, &GazeboRosIMU::ServiceCallback, this);

  accelBiasService = node_handle_->advertiseService(topic_ + "/set_accel_bias", &GazeboRosIMU::SetAccelBiasCallback, this);
  rateBiasService  = node_handle_->advertiseService(topic_ + "/set_rate_bias", &GazeboRosIMU::SetRateBiasCallback, this);

  // setup dynamic_reconfigure servers
  if (!topic_.empty()) {
    dynamic_reconfigure_server_accel_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/accel")));
    dynamic_reconfigure_server_rate_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/rate")));
    dynamic_reconfigure_server_yaw_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/yaw")));
    dynamic_reconfigure_server_accel_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &accelModel, _1, _2));
    dynamic_reconfigure_server_rate_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &rateModel, _1, _2));
    dynamic_reconfigure_server_yaw_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &yawModel, _1, _2));
  }

#ifdef USE_CBQ
  // start custom queue for imu
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::CallbackQueueThread,this ) );
#endif

  Reset();

  // connect Update function
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosIMU::Update, this));
}

void GazeboRosIMU::Reset()
{
  updateTimer.Reset();

  I_v_IB_.setZero();
  I_v_IM_.setZero();

  phi_IM_.setIdentity();
  M_a_IM_.setZero();
  M_w_IM_.setZero();

  accelModel.reset();
  rateModel.reset();
  yawModel.reset();
}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
#if (GAZEBO_MAJOR_VERSION >= 8)
  rateModel.reset(ignition::math::Vector3d(0.0, 0.0, 0.0));
#else
  rateModel.reset(math::Vector3(0.0, 0.0, 0.0));
#endif
  return true;
}

bool GazeboRosIMU::SetAccelBiasCallback(any_hector_gazebo_plugins::SetBias::Request &req, any_hector_gazebo_plugins::SetBias::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
#if (GAZEBO_MAJOR_VERSION >= 8)
  accelModel.reset(ignition::math::Vector3d(req.bias.x, req.bias.y, req.bias.z));
#else
  accelModel.reset(math::Vector3(req.bias.x, req.bias.y, req.bias.z));
#endif
  return true;
}

bool GazeboRosIMU::SetRateBiasCallback(any_hector_gazebo_plugins::SetBias::Request &req, any_hector_gazebo_plugins::SetBias::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
#if (GAZEBO_MAJOR_VERSION >= 8)
  rateModel.reset(ignition::math::Vector3d(req.bias.x, req.bias.y, req.bias.z));
#else
  rateModel.reset(math::Vector3(req.bias.x, req.bias.y, req.bias.z));
#endif
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::Update()
{
  // Get Time Difference dt
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time cur_time = world->SimTime();
#else
  common::Time cur_time = world->GetSimTime();
#endif
  double dt = updateTimer.getTimeSinceLastUpdate().Double();
  boost::mutex::scoped_lock scoped_lock(lock);

  // Get Pose/Orientation
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose_IB = link->WorldPose();
  kindr::HomTransformQuatD T_IB(kindr::Position3D(pose_IB.Pos().X(), pose_IB.Pos().Y(), pose_IB.Pos().Z()),
                                kindr::RotationQuaternionD(pose_IB.Rot().W(), pose_IB.Rot().X(), pose_IB.Rot().Y(), pose_IB.Rot().Z()));
  kindr::RotationQuaternionD phi_IM = T_IB.getRotation() * T_BM_.getRotation();
#else
  math::Pose pose_IB = link->GetWorldPose();
  kindr::HomTransformQuatD T_IB(kindr::Position3D(pose_IB.pos.x, pose_IB.pos.y, pose_IB.pos.z),
                                kindr::RotationQuaternionD(pose_IB.rot.w, pose_IB.rot.x, pose_IB.rot.y, pose_IB.rot.z));
  kindr::RotationQuaternionD phi_IM = T_IB.getRotation() * T_BM_.getRotation();
#endif

  // get Gravity
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d g = world->Gravity();
  gravity = kindr::Acceleration3D(g.X(), g.Y(), g.Z());
#else
  math::Vector3 g = world->GetPhysicsEngine()->GetGravity();
  gravity = kindr::Acceleration3D(g.x, g.y, g.z);
#endif
  double gravity_length = gravity.norm();
  ROS_DEBUG_NAMED("gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.x(), gravity.y(), gravity.z());

  // calculate angular rate of IMU
  // note: link->GetRelativeAngularVel() sometimes return nan?
  kindr::LocalAngularVelocityD M_w_IM; // Angular velocity
  kindr::AngularAcceleration3D M_W_IM; // Angular acceleration

  if (dt > 0.0) {
    M_w_IM = kindr::LocalAngularVelocityD(-phi_IM.inverted().boxMinus(phi_IM_.inverted())/dt);
    M_W_IM = kindr::AngularAcceleration3D( (M_w_IM - M_w_IM_) / dt );
  }
  phi_IM_ = phi_IM;
  M_w_IM_ = M_w_IM;


  // get Acceleration and Angular Rates

  // the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
#if (GAZEBO_MAJOR_VERSION >= 8)  
  ignition::math::Vector3d vel = link->WorldLinearVel(); // get velocity in world frame
  const kindr::Velocity3D I_v_IB = kindr::Velocity3D(vel.X(), vel.Y(), vel.Z());
#else
  math::Vector3 vel = link->GetWorldLinearVel(); // get velocity in world frame
  const kindr::Velocity3D I_v_IB = kindr::Velocity3D(vel.x, vel.y, vel.z);
#endif

  // Calculate Linear acceleration: I_a_IM = I_a_IB + skew(I_W_IM) * Mskew(I_w_IM) * q_IM * M_r_MB
  const kindr::Position3D M_r_BM(T_BM_.getRotation().inverseRotate(T_BM_.getPosition()));
  //const math::Vector3 B_r_BM(T_BM_.getPosition().x(), T_BM_.getPosition().y(), T_BM_.getPosition().z());
  //math::Vector3 temp = link->GetWorldLinearVel(B_r_BM); // get velocity in world frame
  //kindr::Velocity3D I_v_IM(temp.x, temp.y, temp.z);

  const Eigen::Matrix3d M_w_IM_skew = kindr::getSkewMatrixFromVector(M_w_IM.toImplementation());
  if (dt > 0.0) {
    //M_a_IM_ = phi_IM.inverseRotate( kindr::Acceleration3D((I_v_IM - I_v_IM_) / dt) - gravity);
    M_a_IM_ = phi_IM.inverseRotate(kindr::Acceleration3D( (I_v_IB - I_v_IB_) / dt ) - gravity)
    + kindr::Acceleration3D(kindr::getSkewMatrixFromVector(M_W_IM.toImplementation()) * M_r_BM.toImplementation())
    + kindr::Acceleration3D(M_w_IM_skew * (M_w_IM_skew * M_r_BM.toImplementation()));
  }

  I_v_IB_ = I_v_IB;
  //I_v_IM_ = I_v_IM;

  // update sensor models
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d accel(M_a_IM_.x(), M_a_IM_.y(), M_a_IM_.z());
#else
  math::Vector3 accel(M_a_IM_.x(), M_a_IM_.y(), M_a_IM_.z());
#endif
  accel = accelModel(accel, dt);
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d rate(M_w_IM_.x(), M_w_IM_.y(), M_w_IM_.z());
#else
  math::Vector3 rate(M_w_IM_.x(), M_w_IM_.y(), M_w_IM_.z());
#endif
  rate  = rateModel(rate, dt);

  yawModel.update(dt);
#if (GAZEBO_MAJOR_VERSION >= 8)
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Current bias errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                 accelModel.getCurrentBias().X(), accelModel.getCurrentBias().Y(), accelModel.getCurrentBias().Z(),
                 rateModel.getCurrentBias().X(), rateModel.getCurrentBias().Y(), rateModel.getCurrentBias().Z(),
                 yawModel.getCurrentBias());
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Scale errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                 accelModel.getScaleError().X(), accelModel.getScaleError().Y(), accelModel.getScaleError().Z(),
                 rateModel.getScaleError().X(), rateModel.getScaleError().Y(), rateModel.getScaleError().Z(),
                 yawModel.getScaleError());
#else
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Current bias errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                 accelModel.getCurrentBias().x, accelModel.getCurrentBias().y, accelModel.getCurrentBias().z,
                 rateModel.getCurrentBias().x, rateModel.getCurrentBias().y, rateModel.getCurrentBias().z,
                 yawModel.getCurrentBias());
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Scale errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                 accelModel.getScaleError().x, accelModel.getScaleError().y, accelModel.getScaleError().z,
                 rateModel.getScaleError().x, rateModel.getScaleError().y, rateModel.getScaleError().z,
                 yawModel.getScaleError());
#endif

  // apply accelerometer and yaw drift error to orientation (pseudo AHRS)
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Quaterniond phi_IB(T_IB.getRotation().w(), T_IB.getRotation().x(), T_IB.getRotation().y(), T_IB.getRotation().z());
  ignition::math::Vector3d accelDrift = phi_IB.RotateVector(accelModel.getCurrentBias());
  double yawError = yawModel.getCurrentBias();
  kindr::RotationQuaternionD orientationError(
      kindr::RotationQuaternionD(cos(yawError/2), 0.0, 0.0, sin(yawError/2)) *                                         // yaw error
      kindr::RotationQuaternionD(1.0, 0.5 * accelDrift.Y() / gravity_length, 0.5 * -accelDrift.X() / gravity_length, 0.0)  // roll and pitch error
  );
#else
  math::Quaternion phi_IB(T_IB.getRotation().w(), T_IB.getRotation().x(), T_IB.getRotation().y(), T_IB.getRotation().z());
  math::Vector3 accelDrift = phi_IB.RotateVector(accelModel.getCurrentBias());
  double yawError = yawModel.getCurrentBias();
  kindr::RotationQuaternionD orientationError(
      kindr::RotationQuaternionD(cos(yawError/2), 0.0, 0.0, sin(yawError/2)) *                                         // yaw error
      kindr::RotationQuaternionD(1.0, 0.5 * accelDrift.y / gravity_length, 0.5 * -accelDrift.x / gravity_length, 0.0)  // roll and pitch error
  );
#endif
  phi_IM = orientationError * phi_IM;

  // copy data into pose message
  imuMsg.time_.set(cur_time.sec, cur_time.nsec);

  // orientation quaternion
  imuMsg.orientation_.x() = phi_IM.x();
  imuMsg.orientation_.y() = phi_IM.y();
  imuMsg.orientation_.z() = phi_IM.z();
  imuMsg.orientation_.w() = phi_IM.w();

  // pass angular rates
#if (GAZEBO_MAJOR_VERSION >= 8)
  imuMsg.angularVelocity_.x() = rate.X();
  imuMsg.angularVelocity_.y() = rate.Y();
  imuMsg.angularVelocity_.z() = rate.Z();
#else
  imuMsg.angularVelocity_.x() = rate.x;
  imuMsg.angularVelocity_.y() = rate.y;
  imuMsg.angularVelocity_.z() = rate.z;
#endif

  // pass accelerations
#if (GAZEBO_MAJOR_VERSION >= 8)
  imuMsg.linearAcceleration_.x()    = accel.X();
  imuMsg.linearAcceleration_.y()    = accel.Y();
  imuMsg.linearAcceleration_.z()    = accel.Z();
#else
  imuMsg.linearAcceleration_.x()    = accel.x;
  imuMsg.linearAcceleration_.y()    = accel.y;
  imuMsg.linearAcceleration_.z()    = accel.z;
#endif

  // publish to ros
  any_measurements_ros::toRos(imuMsg, rosImuMsg);
  pub_->publishAndSend(imuMsg, rosImuMsg);
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Publishing IMU data at t = %f", cur_time.Double());

  // publish bias
  if (bias_pub_) {
    biasMsg.header.stamp.sec = cur_time.sec;
    biasMsg.header.stamp.nsec = cur_time.nsec;
    biasMsg.orientation.x = orientationError.x();
    biasMsg.orientation.y = orientationError.y();
    biasMsg.orientation.z = orientationError.z();
    biasMsg.orientation.w = orientationError.w();
#if (GAZEBO_MAJOR_VERSION >= 8)
    biasMsg.angular_velocity.x = rateModel.getCurrentBias().X();
    biasMsg.angular_velocity.y = rateModel.getCurrentBias().Y();
    biasMsg.angular_velocity.z = rateModel.getCurrentBias().Z();
    biasMsg.linear_acceleration.x = accelModel.getCurrentBias().X();
    biasMsg.linear_acceleration.y = accelModel.getCurrentBias().Y();
    biasMsg.linear_acceleration.z = accelModel.getCurrentBias().Z();
#else
    biasMsg.angular_velocity.x = rateModel.getCurrentBias().x;
    biasMsg.angular_velocity.y = rateModel.getCurrentBias().y;
    biasMsg.angular_velocity.z = rateModel.getCurrentBias().z;
    biasMsg.linear_acceleration.x = accelModel.getCurrentBias().x;
    biasMsg.linear_acceleration.y = accelModel.getCurrentBias().y;
    biasMsg.linear_acceleration.z = accelModel.getCurrentBias().z;
#endif
    bias_pub_.publish(biasMsg);
  }

  // debug output
#ifdef DEBUG_OUTPUT
  if (debugPublisher) {
    geometry_msgs::PoseStamped debugPose;
    debugPose.header = imuMsg.header;
    debugPose.header.frame_id = "/map";
    debugPose.pose.orientation.w = imuMsg.orientation.w;
    debugPose.pose.orientation.x = imuMsg.orientation.x;
    debugPose.pose.orientation.y = imuMsg.orientation.y;
    debugPose.pose.orientation.z = imuMsg.orientation.z;
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d pose = link->WorldPose();
    debugPose.pose.position.x = pose.Pos().X();
    debugPose.pose.position.y = pose.Pos().Y();
    debugPose.pose.position.z = pose.Pos().Z();
#else
    math::Pose pose = link->GetWorldPose();
    debugPose.pose.position.x = pose.pos.x;
    debugPose.pose.position.y = pose.pos.y;
    debugPose.pose.position.z = pose.pos.z;
#endif
    debugPublisher.publish(debugPose);
  }
#endif // DEBUG_OUTPUT
}

#ifdef USE_CBQ
void GazeboRosIMU::CallbackQueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIMU)

} // namespace gazebo
