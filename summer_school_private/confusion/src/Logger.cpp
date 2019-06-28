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

#include "confusion/Logger.h"


namespace confusion {

Logger::Logger(const std::string &fName) : file_(fName) {
  file_.precision(16);
  startLog();
}

Logger::Logger(const std::string &fName, const State &firstState) : file_(fName) {
  file_.precision(16);
  writeHeader(firstState);
  startLog();
}

Logger::~Logger() {
  file_.close();
}

void Logger::writeBatch(StateVector stateVector) {
  file_ << "BS " << stateVector.size() << "\n";
  for (const auto &state : stateVector) {
    file_ << "ST " << state->t_ << " ";
    for (const auto &parameter : state->parameters_) {
      for (int k = 0; k < parameter.size(); ++k) {
        file_ << parameter.data_[k] << " ";
      }
    }
    file_ << "\n";
  }
  file_ << "BS -1\n";
}

void Logger::writeStaticParameters(StaticParameterVector staticParameterVector, double t) {
  file_ << "SP " << staticParameterVector.size() << "\n";
  for (const auto &parameter : staticParameterVector) {
    file_ << parameter.second.name() << " " << t << " ";
    for (int k = 0; k < parameter.second.size(); ++k) {
      file_ << parameter.second.data_[k] << " ";
    }
    file_ << "\n";
  }
  file_ << "SP -1\n";
}

void Logger::writeResiduals(const Eigen::VectorXd &priorResidual,
                    const std::vector<Eigen::VectorXd> &processResiduals,
                    const std::vector<Eigen::VectorXd> &updateResiduals) {
  writePriorResiduals(priorResidual);
  writeProcessResiduals(processResiduals);
  writeUpdateResiduals(updateResiduals);
}

void Logger::writePriorResiduals(const Eigen::VectorXd &priorResidual) {
  file_ << "PI " << priorResidual.transpose() << "\n";
}

void Logger::writeProcessResiduals(const std::vector<Eigen::VectorXd> &processResiduals) {
  file_ << "BP " << processResiduals.size() << "\n";
  for (const auto &processResidual : processResiduals) {
    file_ << "PO " << processResidual.transpose() << "\n";
  }
  file_ << "BP -1\n";
}

void Logger::writeUpdateResiduals(const std::vector<Eigen::VectorXd> &updateResiduals) {
  file_ << "BU " << updateResiduals.size() << "\n";
  for (const auto &updateResidual : updateResiduals) {
    file_ << "UR " << updateResidual.transpose() << "\n";
  }
  file_ << "BU -1\n";
}

void Logger::writeUserDataVector(const Eigen::VectorXd &userData) {
  file_ << "UU ";
  for (int k = 0; k < userData.rows(); ++k) {
    file_ << userData(k) << " ";
  }
  file_ << "\n";
}

void Logger::startLog() {
  file_ << "Start log\n";
}

void Logger::writeHeader(const State &state) {
  // Write first line which defines structure of state
  file_ << "State names\n";
  for (const Parameter &parameter : state.parameters_) {
    file_ << parameter.size() << " " << parameter.name() << "\n";
  }
}

} // namespace confusion

