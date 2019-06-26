/*
 * DynamicIdentification.hpp
 *
 *  Created on: Oct 27, 2016
 *      Author: Jan Carius, Dario Bellicoso
 */

#pragma once

// eigen
#include <Eigen/Core>

// STL
#include <string>
#include <vector>

namespace romo {

template<typename RobotModel_>
class DynamicIdentification {

 public:
  typedef RobotModel_ RobotModel;
  typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> MeasurementPair;

  explicit DynamicIdentification(const RobotModel& model) : model_(model) { }
  virtual ~DynamicIdentification() = default;

  virtual void addMeasurementPair(const MeasurementPair& measurementPair) = 0;
  virtual bool identify(Eigen::VectorXd& dynamicParams, bool verbose = false) = 0;
  virtual double getOptimizationCost() const = 0;

 protected:
  RobotModel model_;

 private:
  virtual double getTorqueError(const Eigen::VectorXd& dynamicParams) = 0;
  virtual void printDynamicParams(const Eigen::VectorXd& dynamicParams,
                                  const std::string& msg="") const = 0;
  virtual void printMeasurements( const std::vector<MeasurementPair>& measurements) const = 0;
};

} /* namespace romo */
