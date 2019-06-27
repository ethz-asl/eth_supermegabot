/*
 * DynamicIdentificationImpl.tpp
 *
 *  Created on: Oct 27, 2016
 *      Author: Jan Carius, Dario Bellicoso
 */

// Nelder-Mead optimization
#include <robot_utils/math/OptimizationNelderMead.hpp>

//message logger
#include <message_logger/message_logger.hpp>

namespace romo_std {

template<typename RobotModel_>
DynamicIdentificationImpl<RobotModel_>::DynamicIdentificationImpl(const RobotModel& model):
    Base(model), cost_(0.0) {
    numberOfMovableLinks_ = RobotModel::RobotState::getNumberOfJointPositions();
}

template<typename RobotModel_>
void DynamicIdentificationImpl<RobotModel_>::addMeasurementPair(
    const MeasurementPair& measurementPair) {
  if(measurementPair.first.size() != measurementPair.second.size() or
      measurementPair.first.size() != numberOfMovableLinks_){
    MELO_ERROR("[DynamicIdentificationImpl::addMeasurementPair]"
        "Measurement pair of wrong dimension. Not adding.");
    return;
  }
  measurementPairs_.push_back(measurementPair);
}

template<typename RobotModel_>
bool DynamicIdentificationImpl<RobotModel_>::identify(Eigen::VectorXd& dynamicParams, bool verbose) {
  robot_utils::optimization::OptimizationNelderMead optNm;
  Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(numberOfMovableLinks_*numberOfParamsPerLink_);
  this->model_.getGravityDynamicParameters(initialGuess);
  dynamicParams = Eigen::VectorXd::Zero(numberOfMovableLinks_*numberOfParamsPerLink_);

  if (verbose) {
    MELO_INFO_STREAM("[DynamicIdentification::identify] Initial guess");
    this->printDynamicParams(initialGuess, "initial guess");
    MELO_INFO_STREAM("[DynamicIdentification::identify] Starting optimization with the following measurements:");
    this->printMeasurements(measurementPairs_);
  }


  if (!optNm.optimize(cost_,
                      dynamicParams,
                      initialGuess,
                      std::bind(&DynamicIdentificationImpl::getTorqueError, this, std::placeholders::_1),
                      true,
                      100000)) {
    MELO_WARN_STREAM("[DynamicIdentification::identify] Identification failed!");
    return false;
  }

  return true;
}

template<typename RobotModel_>
double DynamicIdentificationImpl<RobotModel_>::getOptimizationCost() const {
  return cost_;
}

template<typename RobotModel_>
double DynamicIdentificationImpl<RobotModel_>::getTorqueError(const Eigen::VectorXd& dynamicParams){
  if(dynamicParams.size() != numberOfMovableLinks_*numberOfParamsPerLink_){
    MELO_ERROR_STREAM("[DynamicIdentificationImpl::getTorqueError] dynamicParams has wrong size (" <<
        dynamicParams.size() << "), expected " << numberOfMovableLinks_*numberOfParamsPerLink_);
  }

  const double w = 1.0;

  double sumOfSquareErrors = 0.0;

  for (auto measurement : measurementPairs_){
    Eigen::VectorXd armGravityTerms = Eigen::VectorXd::Zero(numberOfMovableLinks_);
    this->model_.getJointGravityTermsFromJointConfigurationAndLinkParameters(armGravityTerms,
                                                                             measurement.first,
                                                                             dynamicParams);
    sumOfSquareErrors += w*(measurement.second - armGravityTerms).norm();
  }

  return sumOfSquareErrors;
}

template<typename RobotModel_>
void DynamicIdentificationImpl<RobotModel_>::printDynamicParams(const Eigen::VectorXd& dynamicParams,
                                                                const std::string& msg) const {
  std::cout << "[DynamicIdentification] Printing the dynamic parameters (" << msg << "):"<< std::endl;
  for (unsigned int linkId=0; linkId<numberOfMovableLinks_; linkId++) {
    std::cout << "-----------------------" << std::endl;
    std::cout << "link id: " << linkId << std::endl;
    std::cout << "mass: " << dynamicParams(numberOfParamsPerLink_*linkId) << std::endl;
    std::cout << "k_r_ks: " << dynamicParams.segment<3>(numberOfParamsPerLink_*linkId+1).transpose() << std::endl;
    std::cout << "-----------------------" << std::endl;
    std::cout << std::endl;
  }
}

template<typename RobotModel_>
void DynamicIdentificationImpl<RobotModel_>::printMeasurements(const std::vector<MeasurementPair>& measurements) const {
  std::cout << "[DynamicIdentification] Printing the measurements:" << std::endl;
  for (unsigned int i=0; i<measurements.size(); i++){
    std::cout << "Measurement " << i << ":" << std::endl;
    std::cout << std::left << std::setw(15) << "Position" << std::setw(15) << "Torque" << std::endl;
    MeasurementPair mp = measurements.at(i);
    for(unsigned int j=0; j<mp.first.size(); j++){
      std::cout << std::setw(15) << mp.first(j) << std::setw(15) << mp.second(j) << std::endl;
    }
    std::cout << "------------------------------" << std::endl;
  }
}

} /* namespace romo_std */
