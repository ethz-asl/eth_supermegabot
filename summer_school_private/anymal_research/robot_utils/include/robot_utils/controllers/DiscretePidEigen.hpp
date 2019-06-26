/*
 * PID controller class for use on Eigen-Types with a filter on the derivative term.
 *
 * C(s) = Kp + Ki/s + N*Kd / (1 + N/s)
 *
 * u[k] = a*u[k-1] + b*u[k-2] + c*e[k] + d*e[k-1] + e*e[k-2]
 *
 * a = (2 + N*dt)/(1 + N*dt)
 * b = -1/(1 + N*dt)
 * c = (Kp(1 + N*dt) + Ki*dt*(1 + N*dt) + Kd*N)/(1 + N*dt)
 * d = -(Kp*(2 + N*dt) + Ki*dt + 2*Kd*N)/(1 + N*dt)
 * e = (Kp + Kd*N)/(1 + N*dt)
 *
 */


#pragma once

#include <Eigen/Core>
#include <assert.h>

namespace robot_utils {

/*! Proportional-integral-derivative class for use with eigen types.
 *  The controller acts element-wise.
 */
class DiscretePidEigen{

public:
  typedef Eigen::MatrixXd MatrixXd;

  /*! Constructor.
   *  For arguments, see the init method.
   */
  DiscretePidEigen(const MatrixXd& pGains, const MatrixXd& iGains, const MatrixXd& dGains,
           const MatrixXd& cmdMax, const MatrixXd& cmdMin);

  virtual ~DiscretePidEigen() {};

  /*! Initializes the controller.
   *  All parameters need to have the same dimension, otherwise an error is thrown.
   *  For zero integral action, the respective element in iGains should be set zero.
   *
   * @param[in] pGains  proportional controller gains
   * @param[in] iGains  integrator controller gains
   * @param[in] dGains  derivative controller gains
   * @param[in] cmdMax  maximum output values of the controller
   * @param[in] cmdMin  minimum output values of the controller (must be less than cmdMax)
   */
  void init(const MatrixXd& pGains, const MatrixXd& iGains, const MatrixXd& dGains,
            const MatrixXd& cmdMax, const MatrixXd& cmdMin);

  void setPGains(const MatrixXd& pGains);
  void setIGains(const MatrixXd& iGains);
  void setDGains(const MatrixXd& dGains);
  void setCmdMax(const MatrixXd& cmdMax);
  void setCmdMin(const MatrixXd& cmdMin);
  void setDFilterGain(const double& dFilterGain);

  MatrixXd getPGains() const {return pGains_;}
  MatrixXd getIGains() const {return iGains_;}
  MatrixXd getDGains() const {return dGains_;}
  MatrixXd getCmdMax() const {return cmdMax_;}
  MatrixXd getCmdMin() const {return cmdMin_;}
  MatrixXd getCmd()    const {return cmdLast_;}
  double getDFilterGain() const {return dFilterGain_;}


  /*! Updates the PID controller, should be called each time step.
   *  All parameters need to have the same dimension, otherwise an error is thrown.
   *
   * @param[in]  error    current error = (actual value - desired value) as input to the controller
   * @param[out] command  to be applied to the system
   * @param[in]  dt       control update step (time elapsed from last update call) [s]
   */
  template<typename Derived>
  void update(const Eigen::MatrixBase<Derived>& error, Eigen::MatrixBase<Derived>& command, double dt){
    //note: due to templated member fct, implementation is in header file

    //assertion on matrix sizes
    assert(error.rows() == command.rows());
    assert(error.cols() == command.cols());
    assert(error.rows() == pGains_.rows());
    assert(error.cols() == pGains_.cols());

    if(dt <= 0.0 or !error.allFinite()){
      command.setZero();
      return;
    }
    pErr_ = error;

    double a0, a1, a2;
    MatrixXd b0, b1, b2;
    a0 = (1.0 + dFilterGain_*dt);
    a1 = -(2.0 + dFilterGain_*dt);
    a2 = 1.0;
    b0 = pGains_*(1.0 + dFilterGain_*dt) + iGains_*dt*(1.0 + dFilterGain_*dt) + dGains_*dFilterGain_;
    b1 = -pGains_*(2.0 + dFilterGain_*dt) - iGains_*dt - 2.0*dGains_*dFilterGain_;
    b2 = pGains_ + dGains_*dFilterGain_;

    command = - a1*cmdLast_ - a2*cmdSecondLast_ - b0.cwiseProduct(pErr_) - b1.cwiseProduct(pErrLast_) - b2.cwiseProduct(pErrSecondLast_);
    command /= a0;
    command = 0.5*(command-cmdMin_+(command-cmdMin_).cwiseAbs()) + cmdMin_; //bound from below
    command = cmdMax_ - 0.5*(cmdMax_-command+(cmdMax_-command).cwiseAbs()); //bound from above

    cmdSecondLast_ = cmdLast_;
    cmdLast_ = command;

    pErrSecondLast_ = pErrLast_;
    pErrLast_ = pErr_;
  }

  /*! Resets the PID controller.
   *  All errors get set to zero (including integrator state).
   *  Note that this may cause a jump in the control action.
   */
  void reset();

private:

  double dFilterGain_;
  MatrixXd pErrLast_;
  MatrixXd pErrSecondLast_;
  MatrixXd pErr_;
  MatrixXd dErr_;

  MatrixXd pGains_;
  MatrixXd iGains_;
  MatrixXd dGains_;
  MatrixXd cmdMax_;
  MatrixXd cmdMin_;

  MatrixXd cmdLast_;
  MatrixXd cmdSecondLast_;
};


} //namespace robot_utils
