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

#ifndef INCLUDE_CONFUSION_MEASUREMENTMANAGER_H_
#define INCLUDE_CONFUSION_MEASUREMENTMANAGER_H_

//#define DEBUG_MEAS_MAN

#include <array>
#include <deque>
#include <vector>
#include <mutex>

#include "confusion/State.h"
#include "confusion/StateVector.h"

namespace confusion {

/**
 * Handles the assignment of measurements to states in the MHE problem and triggers the creation of new states
 * when appropriate. Process measurements of each type must be added sequentially in time, and update measurements can
 * be added arbitrarily as desired.
 */
class MeasurementManager {
 public:
  MeasurementManager(int numProcessSensors, int numUpdateSensors);

  void addUpdateMeasurement(std::shared_ptr<UpdateMeasurement> measPtr);

  void addProcessMeasurement(std::shared_ptr<ProcessMeasurement> measPtr);

  /**
   * This is called by the ConFusor to assign new measurements to states and spawn new states when apropriate. It is
   * assumed that process measurements of each type arrive sequentially. States can only be created progressing in
   * time. Returns true if a new state was created.
   */
  bool assignMeasurements(StateVector &stateVector, StaticParameterVector &staticParameters);

  //Empty the measurement buffers
  void reset();

  const size_t numProcessSensors_;
  const size_t numUpdateSensors_;

  std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>> processMeasBuffer_;
  std::vector<std::deque<std::shared_ptr<UpdateMeasurement>>> updateMeasBuffer_;

  std::mutex measMtx;

 private:
  //We need an internal function to allow for an internal recursion without re-acquiring the mutex
  //It is assumed that measurements of each type arrive sequentially
  //States can only be created progressing in time
  //Return true if a new state or measurements were added
  void assignMeasurementsInternal(StateVector &stateVector, StaticParameterVector &staticParameters, bool &res);

  void removeFrontUpdateMeasurement(size_t updateMeasIndex);
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_MEASUREMENTMANAGER_H_ */
