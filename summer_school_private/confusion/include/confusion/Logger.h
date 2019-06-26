/*
 * Copyright 2018 Timothy Sandy
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
 */

#ifndef INCLUDE_CONFUSION_LOGGER_H_
#define INCLUDE_CONFUSION_LOGGER_H_

#include <mutex>
#include <fstream>
#include "confusion/Parameter.h"
#include "confusion/StateVector.h"

namespace confusion {

/**
 * @brief Logger produces a human-readable text log file.
 *
 * If a state is passed to the constructor, there is a header of the form
 * 		State names
 * 		num name
 *		num name
 * 		...
 * where
 *		num is the number of dimensions of this parameter
 *		name is the name of this parameter, state batches are imported into variables of this name if using the provided Matlab script
 *
 * The body of the log file is always in the form
 * 		Start log
 *		ID n1 n2 n3 n4 ... nN
 * 		ID n1 n2 n3 n4 ... nN
 *		...
 * where ID identifies the recorded data
 *      BP: Batch of Process residuals
 *      BS: Batch of States
 *      BU: Batch of Update residuals
 *      PI: PrIor residuals
 *      PO: PrOcess residuals
 *      SP: Static Parameters
 *      ST: STate
 *      UR: Update Residuals
 *      UU: User data vector
 *
 * Batches of x with n entries start with
 *      Bx n
 * continue with the corresponding lines, and end with
 *      Bx -1
 *  e. g. for a state bath the entry would be
 *      BS 3
 *      ST ...
 *      ST...
 *      ST...
 *      BS -1
 *
 * The resulting log file can be imported by /matlab/importConFusionFromText.m
 */
class Logger {
 public:
  Logger(const std::string &fName);

  Logger(const std::string &fName, const State &firstState);

  ~Logger();

  void writeBatch(StateVector stateVector);

  void writeStaticParameters(StaticParameterVector staticParameterVector, double t);

  void writeResiduals(const Eigen::VectorXd &priorResidual,
                      const std::vector<Eigen::VectorXd> &processResiduals,
                      const std::vector<Eigen::VectorXd> &updateResiduals);

  void writePriorResiduals(const Eigen::VectorXd &priorResidual);

  void writeProcessResiduals(const std::vector<Eigen::VectorXd> &processResiduals);

  void writeUpdateResiduals(const std::vector<Eigen::VectorXd> &updateResiduals);

  void writeUserDataVector(const Eigen::VectorXd &userData);

 private:
  void startLog();
  void writeHeader(const State &state);

  std::ofstream file_;
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_LOGGER_H_ */
