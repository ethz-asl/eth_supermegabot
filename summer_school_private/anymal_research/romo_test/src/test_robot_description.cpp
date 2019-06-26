#include <romo_test/ExampleDescription.hpp>
#include <romo_test/ExampleContainersRos.hpp>

#include <gtest/gtest.h>
#include <iostream>

#include <Eigen/Core>

TEST(RobotDescriptionRos, Convert) {
  using ECRos = romo_test::ExampleContainersRos;
  ECRos::ActuatorReadingsRos::type reading;
  reading.read();
  ECRos::ActuatorReadingsRos::msgType readingRos =
      ECRos::ActuatorReadingsRos::ConversionTrait<typename ECRos::ActuatorReadingsRos::type,
                                                  typename ECRos::ActuatorReadingsRos::msgType>::convert(reading);
  readingRos.send();
}

TEST(RobotDescription, LoopKeys) {
  using ED = romo_test::ExampleDescription;

  for( const auto & key : ED::getGeneralizedCoordinateKeys() ) {
    std::cout << "GeneralizedCoordinates -> Name: "<< key.getName() << " Id: " << key.getId() << std::endl;
  }
}

TEST(RobotDescription, MapEnums) {
  using ED = romo_test::ExampleDescription;

  for( const auto & actuatorKey : ED::getActuatorKeys() ) {
    std::cout << "Actuator "<< actuatorKey.getName() << " maps to Joint "
    << ED::mapKeyEnumToKeyName<ED::ActuatorEnum, ED::JointEnum>(actuatorKey.getEnum()) << "!" << std::endl;
    std::cout << "Actuator Id"<< actuatorKey.getId() << " maps to Limb Id "
    << ED::mapKeyEnumToKeyId<ED::ActuatorEnum, ED::LimbEnum>(actuatorKey.getEnum()) << "!" << std::endl;
  }
}

TEST(RobotDescription, FixedSizeEigen) {
  using ED = romo_test::ExampleDescription;

  Eigen::Matrix<std::string, ED::getNumDof(), 1> myVec;

  for( const auto & key : ED::getGeneralizedVelocityKeys() ) {
    myVec(key.getId()) = key.getName();
  }
}