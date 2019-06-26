/*
 * PID controller class for use with Eigen-Types.
 * Based on the PID implementation of the Gazebo Simulator (Open Source Robotics Foundation)
 * See: gazebo->common->PID
 *
 * The below copyright statement is reproduced from gazebo sourcecode:
 *
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */


#pragma once

#include <Eigen/Core>
#include <iostream>

#include <boost/assert.hpp>

namespace robot_utils {

/*! Proportional-integral-derivative class for use with eigen types.
 *  The controller acts element-wise.
 */
class PidEigen{

public:
  typedef Eigen::MatrixXd MatrixXd;

  /*! Constructor.
   *  For arguments, see the init method.
   */
  PidEigen(const MatrixXd& pGains, const MatrixXd& iGains, const MatrixXd& dGains,
           const MatrixXd& iMax, const MatrixXd& iMin,
           const MatrixXd& cmdMax, const MatrixXd& cmdMin);

  virtual ~PidEigen() {};

  /*! Initializes the controller.
   *  All parameters need to have the same dimension, otherwise an error is thrown.
   *  For zero integral action, the respective element in iGains should be set zero.
   *
   * @param[in] pGains  proportional controller gains
   * @param[in] iGains  integrator controller gains
   * @param[in] dGains  derivative controller gains
   * @param[in] iMax    maximum values the integrators may attain
   * @param[in] iMin    maximum values the integrators may attain (must be less than iMax)
   * @param[in] cmdMax  maximum output values of the controller
   * @param[in] cmdMin  minimum output values of the controller (must be less than cmdMax)
   */
  void init(const MatrixXd& pGains, const MatrixXd& iGains, const MatrixXd& dGains,
            const MatrixXd& iMax, const MatrixXd& iMin,
            const MatrixXd& cmdMax, const MatrixXd& cmdMin);

  void setPGains(const MatrixXd& pGains);
  void setIGains(const MatrixXd& iGains);
  void setDGains(const MatrixXd& dGains);
  void setIMax(const MatrixXd& iMax);
  void setIMin(const MatrixXd& iMin);
  void setCmdMax(const MatrixXd& cmdMax);
  void setCmdMin(const MatrixXd& cmdMin);

  MatrixXd getPGains() const {return pGains_;}
  MatrixXd getIGains() const {return iGains_;}
  MatrixXd getDGains() const {return dGains_;}
  MatrixXd getIMax()   const {return iMax_;}
  MatrixXd getIMin()   const {return iMin_;}
  MatrixXd getCmdMax() const {return cmdMax_;}
  MatrixXd getCmdMin() const {return cmdMin_;}
  MatrixXd getCmd()    const {return cmd_;}


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
    BOOST_ASSERT_MSG(error.rows() == command.rows(),
                     ("'error' rows: " + std::to_string(error.rows())
                         + " command rows: " + std::to_string(command.rows())).c_str());
    BOOST_ASSERT_MSG(error.cols() == command.cols(),
                     ("'error' cols: " + std::to_string(error.cols())
                    + " command cols: " + std::to_string(command.cols())).c_str());
    assert(error.rows() == pGains_.rows());
    assert(error.cols() == pGains_.cols());

    if(dt <= 0.0 or !error.allFinite()){
      command.setZero();
      return;
    }
    pErr_ = error;

    MatrixXd pTerm, iTerm, dTerm;

    //proportional contribution
    pTerm = pGains_.cwiseProduct(pErr_);

    //integral contribution
    iErr_ = iErr_ + dt*pErr_;
    iTerm = iGains_.cwiseProduct(iErr_);
    iTerm = 0.5*(iTerm-iMin_+(iTerm-iMin_).cwiseAbs()) + iMin_; //bound from below
    iTerm = iMax_ - 0.5*(iMax_-iTerm+(iMax_-iTerm).cwiseAbs()); //bound from above
    iErr_ = iTerm.cwiseQuotient(iGains_); //correct iErr in case it was cropped above (may produce nan)
    iErr_ = (iErr_.cwiseProduct(iErr_).array() >= 0.0).select(iErr_, 0.0); //removes nan again

    //derivative contribution
    dErr_ = (pErr_-pErrLast_)/dt;
    dTerm = dGains_.cwiseProduct(dErr_);

    //assign command and check limits
    command = -(pTerm + iTerm + dTerm);
    command = 0.5*(command-cmdMin_+(command-cmdMin_).cwiseAbs()) + cmdMin_; //bound from below
    command = cmdMax_ - 0.5*(cmdMax_-command+(cmdMax_-command).cwiseAbs()); //bound from above

    pErrLast_ = pErr_;
  }


  /*! Retreive the current state of the controller.
   *
   *  @param[out] pError  current proporional error
   *  @param[out] iError  current integrator value
   *  @param[out] pError  current error derivative
   */
  void getErrors(MatrixXd& pError, MatrixXd& iError, MatrixXd& dError) const;

  /*! Resets the PID controller.
   *  All errors get set to zero (including integrator state).
   *  Note that this may cause a jump in the control action.
   */
  void reset();

private:

  MatrixXd pErrLast_;
  MatrixXd pErr_;
  MatrixXd iErr_;
  MatrixXd dErr_;

  MatrixXd pGains_;
  MatrixXd iGains_;
  MatrixXd dGains_;
  MatrixXd iMax_;
  MatrixXd iMin_;
  MatrixXd cmdMax_;
  MatrixXd cmdMin_;

  MatrixXd cmd_;
};


} //namespace robot_utils
