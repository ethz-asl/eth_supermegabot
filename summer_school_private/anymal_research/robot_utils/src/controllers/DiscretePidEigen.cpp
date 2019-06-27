#include <robot_utils/controllers/DiscretePidEigen.hpp>

namespace robot_utils {

DiscretePidEigen::DiscretePidEigen(const MatrixXd& pGains,
                                   const MatrixXd& iGains,
                                   const MatrixXd& dGains,
                                   const MatrixXd& cmdMax,
                                   const MatrixXd& cmdMin) {
  init(pGains, iGains, dGains, cmdMax, cmdMin);
}

void DiscretePidEigen::init(const MatrixXd& pGains, const MatrixXd& iGains,
                            const MatrixXd& dGains, const MatrixXd& cmdMax,
                            const MatrixXd& cmdMin) {
  //assert same sizes
  assert(pGains.rows() == iGains.rows());
  assert(pGains.cols() == iGains.cols());
  assert(pGains.rows() == dGains.rows());
  assert(pGains.cols() == dGains.cols());
  assert(pGains.rows() == cmdMax.rows());
  assert(pGains.cols() == cmdMax.cols());
  assert(pGains.rows() == cmdMin.rows());
  assert(pGains.cols() == cmdMin.cols());

  //assert min-max relations
  assert((cmdMax.cwiseMax(cmdMin) - cmdMax).isApproxToConstant(0.0, 1e-9));

  //assign controller settings
  pGains_ = pGains;
  iGains_ = iGains;
  dGains_ = dGains;
  cmdMax_ = cmdMax;
  cmdMin_ = cmdMin;
  dFilterGain_ = 0.0;

  reset();
}

void DiscretePidEigen::setPGains(const MatrixXd& pGains){
  assert(pGains.rows() == pGains_.rows());
  assert(pGains.cols() == pGains_.cols());
  pGains_ = pGains;
}

void DiscretePidEigen::setIGains(const MatrixXd& iGains){
  assert(iGains.rows() == iGains_.rows());
  assert(iGains.cols() == iGains_.cols());
  iGains_ = iGains;
}

void DiscretePidEigen::setDGains(const MatrixXd& dGains){
  assert(dGains.rows() == dGains_.rows());
  assert(dGains.cols() == dGains_.cols());
  dGains_ = dGains;
}

void DiscretePidEigen::setCmdMax(const MatrixXd& cmdMax){
  assert(cmdMax.rows() == cmdMax_.rows());
  assert(cmdMax.cols() == cmdMax_.cols());
  cmdMax_ = cmdMax;
}

void DiscretePidEigen::setCmdMin(const MatrixXd& cmdMin){
  assert(cmdMin.rows() == cmdMin_.rows());
  assert(cmdMin.cols() == cmdMin_.cols());
  cmdMin_ = cmdMin;
}

void DiscretePidEigen::setDFilterGain(const double& dFilterGain){
  dFilterGain_ = dFilterGain;
}

void DiscretePidEigen::reset(){
  //assignment to pGains necessary to ensure correct sizes
  pErr_ = pGains_;
  pErr_.setZero();
  pErrLast_ = pGains_;
  pErrLast_.setZero();
  pErrSecondLast_ = pGains_;
  pErrSecondLast_.setZero();
  dErr_ = pGains_;
  dErr_.setZero();
  cmdLast_ = pGains_;
  cmdLast_.setZero();
  cmdSecondLast_ = pGains_;
  cmdSecondLast_.setZero();
}


} //namespace robot_utils
