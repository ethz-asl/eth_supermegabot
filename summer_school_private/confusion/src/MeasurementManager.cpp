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

#include "confusion/MeasurementManager.h"

namespace confusion {

MeasurementManager::MeasurementManager(int numProcessSensors, int numUpdateSensors) :
    numProcessSensors_(numProcessSensors), numUpdateSensors_(numUpdateSensors),
    processMeasBuffer_(numProcessSensors), updateMeasBuffer_(numUpdateSensors) {
#ifdef DEBUG_MEAS_MAN
  std::cout << "MeasurementManager created for " << numProcessSensors <<
                " process sensors and " << numUpdateSensors << std::endl;
#endif
}

void MeasurementManager::addUpdateMeasurement(std::shared_ptr<UpdateMeasurement> measPtr) {
  if (measPtr->measType() < numUpdateSensors_) {
    std::lock_guard<std::mutex> lock(measMtx);
    updateMeasBuffer_[measPtr->measType()].push_back(measPtr);
  }
}

void MeasurementManager::addProcessMeasurement(std::shared_ptr<ProcessMeasurement> measPtr) {
  if (measPtr->measType() < numProcessSensors_) {
    std::lock_guard<std::mutex> lock(measMtx);
    processMeasBuffer_[measPtr->measType()].push_back(measPtr);
  }
}

void MeasurementManager::assignMeasurementsInternal(StateVector &stateVector,
                                                    StaticParameterVector &staticParameters,
                                                    bool &res) {
  //Initialize the first state from update measurements
  //Wait until there has been at least one of each process type received before creating the first state
  if (!stateVector.firstStateInitialized) {
//#ifdef DEBUG_MEAS_MAN
//			std::cout << "assignMeasurements: trying to initialize the first state" << std::endl;
//#endif
    double tProcessFront;
    bool processMeasFound = false;
    for (int i = 0; i < numProcessSensors_; ++i) {
      if (!processMeasBuffer_[i].empty() && (!processMeasFound || processMeasBuffer_[i].front()->t() < tProcessFront)) {
        tProcessFront = processMeasBuffer_[i].front()->t();
        processMeasFound = true;
      }
    }

    if (!processMeasFound)
      return; //No process measurements received so far

    //Remove update measurements that come before the first process measurement
    //This means we have to wait until there is an update measurement following the earliest process measurement
    bool validUpdateMeasFound = false;
    for (int i = 0; i < numUpdateSensors_; ++i) {
      if (updateMeasBuffer_[i].empty())
        continue;
      else if (updateMeasBuffer_[i].front()->shouldBeDropped())
        removeFrontUpdateMeasurement(i);

      while (updateMeasBuffer_[i].front()->t() < tProcessFront) {
        if (updateMeasBuffer_[i].size() > 1)
          removeFrontUpdateMeasurement(i);
        else
          break;
      }
      if (updateMeasBuffer_[i].front()->t() >= tProcessFront) {
        validUpdateMeasFound = true;
#ifdef DEBUG_MEAS_MAN
        std::cout << "Valid first update meas found at " << updateMeasBuffer_[i].front()->t() <<
                            ". tProcessFront=" << tProcessFront << std::endl;
#endif
      }
    }

    //If an update measurement was not found following the first process meas, wait for more measurements
    if (!validUpdateMeasFound) {
#ifdef DEBUG_MEAS_MAN
      std::cout << "Valid update measurement not yet found..." << std::endl;
#endif
      return;
    }

    //Create the first state
    //todo Need something fancier here. Might want to wait to initialize the state if there are multiple update measurements per state?
    for (int i = 0; i < numUpdateSensors_; ++i) {
      if (!updateMeasBuffer_[i].empty()) {
        bool res = stateVector.states_.front()->initFirstState(processMeasBuffer_, updateMeasBuffer_, staticParameters);
#ifdef DEBUG_MEAS_MAN
        std::cout << "initFirstState returned " << res << std::endl;
#endif
        if (res) {
          stateVector.firstStateInitialized = true;
          res = true;
        }
        break;
      }
    }

    //If the first state wasn't initialized, we need to wait for the arrival of more measurements
    if (!stateVector.firstStateInitialized) {
      return;
    }
  }

  //Add process measurements
  bool addedProcessMeas = false;
  if (stateVector.states_.size() > 1) {
#ifdef DEBUG_MEAS_MAN
    std::cout << "assignMeasurements: assigning process measurements. t_state_back=" << stateVector.back()->t() << std::endl;
            for (int i=0; i<numProcessSensors_; ++i) {
                if (processMeasBuffer_[i].empty())
                    std::cout << "process " << i << " is empty" << std::endl;
                else
                    std::cout << "process " << i << ": t_state=" << stateVector.states_[stateVector.processMeasStateIndex[i]]->t() <<
                            "; t_meas_front=" << processMeasBuffer_[i].front()->t() <<
                            "; t_meas_back=" << processMeasBuffer_[i].back()->t() << std::endl;
            }
#endif
    for (int i = 0; i < numProcessSensors_; ++i) {
      //Throw out the measurement if it is too old. This should never happen.
      while (processMeasBuffer_[i].size() > 1
          && stateVector.states_[stateVector.processMeasStateIndex[i]]->t() >= processMeasBuffer_[i][1]->t()) {
//						if (stateVector.states_.size() > 1) //Don't print the warning for the first state because it is normal to throw out the old process measurements then
        std::cout << "Process measurement is behind the current state. processMeasStateIndex="
                  << stateVector.processMeasStateIndex[i] <<
                  ". Throwing it out. t_state-t_meas="
                  << stateVector.states_[stateVector.processMeasStateIndex[i]]->t() - processMeasBuffer_[i].front()->t()
                  << std::endl;
        processMeasBuffer_[i].pop_front();
      }

      //Assign measurements to the current state
      //Only assign process measurements when the next state has been created
      //because we need to know when the next state will occur to assign the measurements properly
      if (stateVector.processMeasStateIndex[i] < stateVector.states_.size() - 1) {
        while (processMeasBuffer_[i].size() > 1) {
          bool moveToNextState = false;
          if (stateVector.states_[stateVector.processMeasStateIndex[i] + 1]->t() >= processMeasBuffer_[i][1]->t()) {
            //Assign the measurement to the current state
            stateVector.states_[stateVector.processMeasStateIndex[i]]->addProcessMeas(processMeasBuffer_[i].front());
            processMeasBuffer_[i].pop_front();
            addedProcessMeas = true;
          } else if (stateVector.states_[stateVector.processMeasStateIndex[i] + 1]->t()
              > processMeasBuffer_[i].front()->t()) {
            //The front measurement is not aligned with the next state so the ProcessChains should overlap.
            //Add the measurement to the current state, but save it to assign to the next state later
            stateVector.states_[stateVector.processMeasStateIndex[i]]->addProcessMeas(processMeasBuffer_[i].front());
            addedProcessMeas = true;
            moveToNextState = true;
          } else {
            //The process meas is aligned with the next state. We can move to the next state.
            moveToNextState = true;
          }

          if (moveToNextState) {
            if (!stateVector.states_[stateVector.processMeasStateIndex[i]]->processChains_[i]->measurements_.empty()) {
              //The processChain for this state state is ready for cost computations
              stateVector.states_[stateVector.processMeasStateIndex[i]]->processChains_[i]->assignTimes(
                  stateVector.states_[stateVector.processMeasStateIndex[i]]->t(),
                  stateVector.states_[stateVector.processMeasStateIndex[i] + 1]->t());
#ifdef DEBUG_MEAS_MAN
              std::cout << "Finished applying process meas type " << i << " to state. Assigned measurements are from t=" <<
                                        stateVector.states_[stateVector.processMeasStateIndex[i]]->processChains_[i]->measurements_.front()->t() - stateVector.states_[stateVector.processMeasStateIndex[i]]->t() << " to " <<
                                        stateVector.states_[stateVector.processMeasStateIndex[i]]->processChains_[i]->measurements_.back()->t() - stateVector.states_[stateVector.processMeasStateIndex[i]]->t() <<
                                        " relative to the state time" << std::endl;
#endif
            } else {
              std::cout << "No process measurements of type " << i << " were assigned to the state at t="
                        << stateVector.states_[stateVector.processMeasStateIndex[i]]->t() << std::endl;
            }
            ++stateVector.processMeasStateIndex[i];

            if (stateVector.processMeasStateIndex[i] >= stateVector.states_.size() - 1)
              break;
          }
        }
      }
    }
  } else if (!stateVector.states_.empty()) {
    for (int i = 0; i < numProcessSensors_; ++i) {
      //Throw out measurements older than the first state except for the last one before
      while (processMeasBuffer_[i].size() > 1 && stateVector.states_.front()->t() >= processMeasBuffer_[i][1]->t()) {
        processMeasBuffer_[i].pop_front();
      }
    }
  }
#ifdef DEBUG_MEAS_MAN
  std::cout << "addedProcessMeas  = " << addedProcessMeas << ". t_process0_front=" << processMeasBuffer_[0].front()->t() << ". processBufferSize=" << processMeasBuffer_[0].size() << std::endl;
#endif
#ifdef DEBUG_MEAS_MAN
  std::cout << "assignMeasurements: assigning update measurements. t_state_back=" << stateVector.back()->t() << "; num_states=" << stateVector.size() << std::endl;
        for (int i=0; i<numUpdateSensors_; ++i) {
            if (updateMeasBuffer_[i].empty())
                std::cout << "update " << i << " is empty, state index = " << stateVector.updateMeasStateIndex[i] << std::endl;
            else {
                std::cout << "update " << i << ", state index " << stateVector.updateMeasStateIndex[i] << std::endl;
                std::cout << "t_state=" << stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() <<
                    "; t_meas_front=" << updateMeasBuffer_[i].front()->t() <<
                    "; t_meas_back=" << updateMeasBuffer_[i].back()->t() << std::endl;
            }
        }
#endif

  //Assign update measurements to the states
  for (int i = 0; i < numUpdateSensors_; ++i) {
    if (!updateMeasBuffer_[i].empty() && updateMeasBuffer_[i].front()->shouldBeDropped()) {
      removeFrontUpdateMeasurement(i);
    }

    while (!updateMeasBuffer_[i].empty()) {
      //Move the state index at or before the update measurement. Drop the measurement if it is too old.
      if (stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() > updateMeasBuffer_[i].front()->t()) {
        if (stateVector.states_.front()->t() > updateMeasBuffer_[i].front()->t()) {
          std::cout << "Update measurement of type " << i << " is older than the oldest state. Throwing it out. "
                       "t_state=" << stateVector.states_.front()->t() <<
                    "; t_meas=" << updateMeasBuffer_[i].front()->t() << std::endl;
          removeFrontUpdateMeasurement(i);
          continue;
        }

        //Reset the state index for this update measurement
        stateVector.updateMeasStateIndex[i] = 0;
      }

      //March through the states forward in time up to the time of the measurement
      while (stateVector.updateMeasStateIndex[i] < stateVector.states_.size() - 1 &&
          stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() < updateMeasBuffer_[i].front()->t()) {
        ++stateVector.updateMeasStateIndex[i];
      }

      //Add the measurement if the timestamps agree
      if (stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() == updateMeasBuffer_[i].front()->t()) {
        if (stateVector.states_[stateVector.updateMeasStateIndex[i]]->addUpdateMeas(updateMeasBuffer_[i].front(),
                                                                                    staticParameters)) {
#ifdef DEBUG_MEAS_MAN
          std::cout << "Added update measurement type " << i << " at t=" <<
                                updateMeasBuffer_[i].front()->t() << " to state at t=" <<
                                stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() << std::endl;
#endif
        }
        removeFrontUpdateMeasurement(i);

        continue;
      }
        //todo Instead, create an intermediate state here
      else if (stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() > updateMeasBuffer_[i].front()->t()) {
        //Try to create an additional state, using this update measurement, between two existing ones
        std::shared_ptr<State> newStatePtr =
            stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->createNextState(updateMeasBuffer_[i].front(), staticParameters);

        if (newStatePtr) {
          bool newStateOk = true;

          //Make sure the timestamp of the new state makes sense
          if (newStatePtr->t() > stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->t() &&
              newStatePtr->t() < stateVector.states_[stateVector.updateMeasStateIndex[i]]->t()) {
            std::cout << "Creating intermediate state at t=" << newStatePtr->t() << " t_state_before=" << stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->t() << ", t_state_after=" << stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() << std::endl;

            //Split the process chains of the preceding state up to the new state and assign the rest of the process measurements to the new state
            for (size_t j=0; j<numProcessSensors_; ++j) {
              std::cout << "Prev state has process meas type " << j << " from " << stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.front()->t() << " to "
                        << stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.back()->t() << std::endl;
              if (!stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.empty()) {
                bool prevChainIsReady = stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->ready();

                //Get the process measurement at or directly before the new state timestamp
                size_t processMeasIndex = 0;
                while (processMeasIndex+1 < stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.size() &&
                    stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_[processMeasIndex+1]->t() <= newStatePtr->t()) {
                  ++processMeasIndex;
                }

                //Assign the appropriate process measurements to the new state
                if (processMeasIndex+1 >= stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.size()) {
                  //There aren't any measurements to assign from the previous state
                  if (stateVector.states_[stateVector.updateMeasStateIndex[i]]->processChains_[j]->ready() &&
                      !stateVector.states_[stateVector.updateMeasStateIndex[i]]->processChains_[j]->measurements_.empty()) {
                    //The new state should only copy in the first process measurement from the following state
                    std::cout << "Copying measurement type " << j << " at t=" << stateVector.states_[stateVector.updateMeasStateIndex[i]]->processChains_[j]->measurements_.front()->t() << " to the intermediate state" << std::endl;
                    newStatePtr->processChains_[j]->measurements_.push_back(stateVector.states_[stateVector.updateMeasStateIndex[i]]->processChains_[j]->measurements_.front());
                  }
                  else
                    //todo Temp for debugging
                    std::cout << "\n\nInteresting case in MeasurementManager!\n\n" << std::endl;

//                  if (prevChainIsReady) {
//                    std::cout
//                        << "ERROR: Process measurements type " << j<< " don't make sense while inserting a new state. Aborting."
//                        << std::endl;
//                    std::cout << "Prev state has process meas from " << stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.front()->t() << " to "
//                        << stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.back()->t() << ". New state at t=" << newStatePtr->t()
//                        << ". Following existing state at t=" << stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() << std::endl;
//                    abort();
//                  }
                }
                else {
                  // Copy the trailing chunk of process meas to the new state
                  newStatePtr->processChains_[j]->measurements_.insert(newStatePtr->processChains_[j]->measurements_.begin(),
                      stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.begin()+processMeasIndex,
                      stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.end());
                  std::cout << "New intermediate state adding process meas type " << j << " from " <<
                      newStatePtr->processChains_[j]->measurements_.front()->t() << " to " <<
                      newStatePtr->processChains_[j]->measurements_.back()->t() << ". prevChainIsReady=" << prevChainIsReady << std::endl;

                  if (prevChainIsReady) {
                    //The new state's process chain should also be set as ready from the measurements previously assigned to the leading state
                    newStatePtr->processChains_[j]->assignTimes(newStatePtr->t(), stateVector.states_[stateVector.updateMeasStateIndex[i]]->t());
                  }

                  //Remove the appropriate process measurements from the previous state
                  size_t firstMeasToRemoveIndex = processMeasIndex;
                  if (newStatePtr->t() > stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_[processMeasIndex]->t()) {
                    //The closest preceding process meas needs to be assigned to both the previous and new states
                    ++firstMeasToRemoveIndex;
                  }
                  stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.erase(
                      stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.begin()+firstMeasToRemoveIndex,
                      stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->measurements_.end());

                  //Update the time boundaries for the previous process chain
                  stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->processChains_[j]->assignTimes(
                      stateVector.states_[stateVector.updateMeasStateIndex[i]-1]->t(), newStatePtr->t());
                }
              }
            }

            //Add the new state and assign the update measurement
            stateVector.insertState(newStatePtr, stateVector.updateMeasStateIndex[i]);
//#ifdef DEBUG_MEAS_MAN
            std::cout << "Added an intermediate state at " << newStatePtr->t() << " for update measurement type " << i << std::endl;
//#endif
            if (stateVector.states_[stateVector.updateMeasStateIndex[i]]->addUpdateMeas(updateMeasBuffer_[i].front(),
                                                                                        staticParameters)) {
//#ifdef DEBUG_MEAS_MAN
              std::cout << "Added update measurement type " << i << " at t=" <<
                                updateMeasBuffer_[i].front()->t() << " to state at t=" <<
                                stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() << std::endl;
//#endif
            }
          }
          else {
            std::cout << "WARNING: Intermediate state created while assigning measurements which has an invalid timestamp!? Dropping the new state" << std::endl;
          }


        }

//        std::cout << "WARNING: Update measurement type " << i
//                  << " received which is not aligned in time with already created "
//                     "states. Dropping the measurement. t_state_after="
//                  << stateVector.states_[stateVector.updateMeasStateIndex[i]]->t() <<
//                  "; t_meas=" << updateMeasBuffer_[i].front()->t() << std::endl;
        updateMeasBuffer_[i].pop_front();
        continue;
      }

      //Otherwise keep the measurement and wait
      break;
    }
  }

  //Try to add a new state if a more recent update measurement has been received
  for (int i = 0; i < numUpdateSensors_; ++i) {
    if (!updateMeasBuffer_[i].empty() && stateVector.states_.back()->t() < updateMeasBuffer_[i].front()->t()) {
      std::shared_ptr<State> newStatePtr =
          stateVector.states_.back()->createNextState(processMeasBuffer_, updateMeasBuffer_, staticParameters);
      if (newStatePtr) {
        stateVector.pushState(newStatePtr);
        res = true;
#ifdef DEBUG_MEAS_MAN
        std::cout << "Added new state at t=" << newStatePtr->t() << ", i=" << i << ", stateIndex=" << stateVector.updateMeasStateIndex[i] << std::endl;
#endif
        ++stateVector.updateMeasStateIndex[i];

        //Recursively add new measurements with the new state
//						addMeasurements(processMeasBuffer_, updateMeasBuffer_);
        assignMeasurementsInternal(stateVector, staticParameters, res);

        return;
      } else {
#ifdef DEBUG_MEAS_MAN
        std::cout << "Attempt to createNextState was not accepted" << std::endl;
#endif
        return;
      }
    }
  }
#ifdef DEBUG_MEAS_MAN
  std::cout << "assignMeasurements returning " << res << std::endl;
#endif

  return;
}

/**
 * This is called by the ConFusor to assign new measurements to states and spawn new states when apropriate. It is
 * assumed that process measurements of each type arrive sequentially. States can only be created progressing in
 * time. Returns true if a new state was created.
 */
bool MeasurementManager::assignMeasurements(StateVector &stateVector, StaticParameterVector &staticParameters) {
  bool res = false;
  std::lock_guard<std::mutex> lock(measMtx);

  assignMeasurementsInternal(stateVector, staticParameters, res);

  return res;
}

//Empty the measurement buffers
void MeasurementManager::reset() {
  std::lock_guard<std::mutex> lock(measMtx);

  for (int i = 0; i < numProcessSensors_; ++i)
    processMeasBuffer_[i].clear();
  for (int i = 0; i < numUpdateSensors_; ++i)
    updateMeasBuffer_[i].clear();
#ifdef DEBUG_MEAS_MAN
  std::cout << "Reset the MeasurementManager" << std::endl;
#endif
}

void MeasurementManager::removeFrontUpdateMeasurement(size_t updateMeasIndex) {
  updateMeasBuffer_[updateMeasIndex].pop_front();

  //Always check if the front meas should be dropped before doing anything else with it
  while (!updateMeasBuffer_[updateMeasIndex].empty() && updateMeasBuffer_[updateMeasIndex].front()->shouldBeDropped()) {
    std::cout << "MeasurementManager dropping dropped measurement at t=" << updateMeasBuffer_[updateMeasIndex].front()->t() <<
              "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    updateMeasBuffer_[updateMeasIndex].pop_front();
  }
}

} // namespace confusion