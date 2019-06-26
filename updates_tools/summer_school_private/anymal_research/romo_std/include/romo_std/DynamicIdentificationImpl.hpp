/*
 * DynamicIdentificationImpl.hpp
 *
 *  Created on: Oct 27, 2016
 *      Author: Jan Carius, Dario Bellicoso
 */

#pragma once

// romo
#include <romo/DynamicIdentification.hpp>
#include <romo/RobotModel.hpp>

// eigen
#include <Eigen/Core>

namespace romo_std {

template<typename RobotModel_>
class DynamicIdentificationImpl :
    public romo::DynamicIdentification<RobotModel_> {

public:
  using Base = romo::DynamicIdentification<RobotModel_>;
  typedef typename Base::RobotModel RobotModel;
  typedef typename Base::MeasurementPair MeasurementPair;

  explicit DynamicIdentificationImpl(const RobotModel& model);
  virtual ~DynamicIdentificationImpl() {}

  /*
   * Add a measurement pair of the form
   * pair.first : joint positions
   * pair.second: joint torques
   */
  virtual void addMeasurementPair(const MeasurementPair& measurementPair);

  /*
   * runs the identification procedure (optimization) and assigns
   * the optimal dynamicParams
   */
  virtual bool identify(Eigen::VectorXd& dynamicParams, bool verbose = false);
  virtual double getOptimizationCost() const;
  virtual void printDynamicParams(const Eigen::VectorXd& dynamicParams,
                                  const std::string& msg="") const;
  virtual void printMeasurements(const std::vector<MeasurementPair>& measurements) const;

 private:
  virtual double getTorqueError(const Eigen::VectorXd& dynamicParams);

  static constexpr unsigned int numberOfParamsPerLink_ = 4u;
  unsigned int numberOfMovableLinks_;

  std::vector<MeasurementPair> measurementPairs_;
  double cost_;

};

} /* namespace romo_std */

//include the implementation
#include "DynamicIdentificationImpl.tpp"
