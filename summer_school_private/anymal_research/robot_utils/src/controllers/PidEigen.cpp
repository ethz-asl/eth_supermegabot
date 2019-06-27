#include <robot_utils/controllers/PidEigen.hpp>

namespace robot_utils {

PidEigen::PidEigen(const MatrixXd& pGains, const MatrixXd& iGains, const MatrixXd& dGains,
                   const MatrixXd& iMax, const MatrixXd& iMin,
                   const MatrixXd& cmdMax, const MatrixXd& cmdMin){
  init(pGains, iGains, dGains, iMax, iMin, cmdMax, cmdMin);
}

void PidEigen::init(const MatrixXd& pGains, const MatrixXd& iGains, const MatrixXd& dGains,
                    const MatrixXd& iMax, const MatrixXd& iMin,
                    const MatrixXd& cmdMax, const MatrixXd& cmdMin){
  //assert same sizes
  assert(pGains.rows() == iGains.rows());
  assert(pGains.cols() == iGains.cols());
  assert(pGains.rows() == dGains.rows());
  assert(pGains.cols() == dGains.cols());
  assert(pGains.rows() == iMax.rows());
  assert(pGains.cols() == iMax.cols());
  assert(pGains.rows() == iMin.rows());
  assert(pGains.cols() == iMin.cols());
  assert(pGains.rows() == cmdMax.rows());
  assert(pGains.cols() == cmdMax.cols());
  assert(pGains.rows() == cmdMin.rows());
  assert(pGains.cols() == cmdMin.cols());

  //assert min-max relations
  assert((iMax.cwiseMax(iMin)-iMax).isApproxToConstant(0.0, 1e-9));
  assert((cmdMax.cwiseMax(cmdMin)-cmdMax).isApproxToConstant(0.0, 1e-9));

  //assign controller settings
  pGains_ = pGains;
  iGains_ = iGains;
  dGains_ = dGains;
  iMax_   = iMax;
  iMin_   = iMin;
  cmdMax_ = cmdMax;
  cmdMin_ = cmdMin;

  reset();
}

void PidEigen::setPGains(const MatrixXd& pGains){
  assert(pGains.rows() == pGains_.rows());
  assert(pGains.cols() == pGains_.cols());
  pGains_ = pGains;
}

void PidEigen::setIGains(const MatrixXd& iGains){
  assert(iGains.rows() == iGains_.rows());
  assert(iGains.cols() == iGains_.cols());
  iGains_ = iGains;
}

void PidEigen::setDGains(const MatrixXd& dGains){
  assert(dGains.rows() == dGains_.rows());
  assert(dGains.cols() == dGains_.cols());
  dGains_ = dGains;
}

void PidEigen::setIMax(const MatrixXd& iMax){
  assert(iMax.rows() == iMax_.rows());
  assert(iMax.cols() == iMax_.cols());
  iMax_ = iMax;
}

void PidEigen::setIMin(const MatrixXd& iMin){
  assert(iMin.rows() == iMin_.rows());
  assert(iMin.cols() == iMin_.cols());
  iMin_ = iMin;
}

void PidEigen::setCmdMax(const MatrixXd& cmdMax){
  assert(cmdMax.rows() == cmdMax_.rows());
  assert(cmdMax.cols() == cmdMax_.cols());
  cmdMax_ = cmdMax;
}

void PidEigen::setCmdMin(const MatrixXd& cmdMin){
  assert(cmdMin.rows() == cmdMin_.rows());
  assert(cmdMin.cols() == cmdMin_.cols());
  cmdMin_ = cmdMin;
}

void PidEigen::getErrors(MatrixXd& pError, MatrixXd& iError, MatrixXd& dError) const{
  pError = pErr_;
  iError = iErr_;
  dError = dErr_;
}

void PidEigen::reset(){
  //assignment to pGains necessary to ensure correct sizes
  pErr_ = pGains_;
  pErr_.setZero();
  pErrLast_ = pGains_;
  pErrLast_.setZero();
  iErr_ = pGains_;
  iErr_.setZero();
  dErr_ = pGains_;
  dErr_.setZero();
  cmd_ = pGains_;
  cmd_.setZero();
}


} //namespace robot_utils
