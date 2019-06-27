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

#ifndef INCLUDE_CONFUSION_STATEVECTOR_H_
#define INCLUDE_CONFUSION_STATEVECTOR_H_

#include <memory>
#include <deque>
#include "confusion/ProcessChain.h"
#include "confusion/UpdateMeasurement.h"
#include "confusion/State.h"

namespace confusion {

//todo Support adding a state in the middle of the batch
//todo Move the measurement assignment to the measurement manager. That is more intuitive.

class StateVector {
public:
	//Pass a pointer to the first derived state. That is the way that this class knows what kind of state is being used.
	StateVector(std::shared_ptr<State> firstStatePtr):
			numProcessSensors_(firstStatePtr->numProcessSensors_),
			numUpdateSensors_(firstStatePtr->numUpdateSensors_),
      updateMeasStateIndex(firstStatePtr->numUpdateSensors_),
			processMeasStateIndex(firstStatePtr->numProcessSensors_) {
		pushState(firstStatePtr);
		for (size_t i=0; i<firstStatePtr->numProcessSensors_; ++i) processMeasStateIndex[i] = 0;
		for (size_t i=0; i<firstStatePtr->numUpdateSensors_; ++i) updateMeasStateIndex[i] = 0;
	}

	//This is used in BatchFusor to copy in an existing state trajectory
	//todo You cannot add more states and measurements when created this way
	StateVector(StateVector& stateVector):
			states_(stateVector.states_), firstStateInitialized(stateVector.firstStateInitialized),
			numProcessSensors_(stateVector.numProcessSensors_), numUpdateSensors_(stateVector.numUpdateSensors_) { }

	//todo Do I really need this? It is used to store states from a ConFusor for use later.
	//With this constructor, a lot of the functionality of the class is lost
	StateVector(const size_t numProcessSensors, const size_t numUpdateSensors):
		numProcessSensors_(numProcessSensors), numUpdateSensors_(numUpdateSensors) { }

	void pushState(const std::shared_ptr<State>& newState) {
		states_.push_back(newState);
//std::cout << "Created new state at " << newState->t() << std::endl;
	}

	/**
	 * Insert a new state in the middle of the current active batch
	 * @param newState Pointer to the new state
	 * @param index Position at which the new state should be added (e.g. to add a state between the states at indices 2 and 3, set this to 3)
	 */
	void insertState(const std::shared_ptr<State>& newState, size_t index) {
	  if (index == 0 || index >= states_.size()) {
	    std::cout << "WARNING: StateVector::insertState requested for a position not inside of the current batch. Will not insert the state." << std::endl;
	    return;
	  }

	  states_.insert(states_.begin()+index, newState);

      //We additionally need to shift the measurement assignment indices
      for (auto& binIndex: updateMeasStateIndex) {
        if (binIndex > index)
          ++binIndex;
      }
      for (auto& binIndex: processMeasStateIndex) {
        if (binIndex > index)
          ++binIndex;
      }
	}

	void popState() {
//std::cout << "Removed state at " << states_.front()->t() << std::endl;
		states_.pop_front();

		//We additionally need to shift the measurement assignment indices
		for (auto& binIndex: updateMeasStateIndex) {
			if (binIndex > 0)
				--binIndex;
		}
		for (auto& binIndex: processMeasStateIndex) {
			if (binIndex > 0)
				--binIndex;
		}
	}

	void removeState(size_t index) {
		states_.erase(states_.begin() + index);

		//Shift the measurement assignment indices if they come after the removed state
		for (auto& binIndex: updateMeasStateIndex) {
			if (binIndex > 0 && binIndex >= index)
				--binIndex;
		}
		for (auto& binIndex: processMeasStateIndex) {
			if (binIndex > 0 && binIndex >= index)
				--binIndex;
		}
	}

	//todo Don't immediately return when there is a problem detected
	bool check() {
		bool res = true;
		//todo Switch to iterator-based loops
		for (size_t i=0; i<numProcessSensors_; ++i) {
			for (size_t j=0; j<states_.size()-1; ++j) {
//				if (!states_[j]->processChains_[i]->initialized_) {
//					std::cout << "Process measurement " << i << " for the state at t=" << states_[j]->t() << " is not initialized!" << std::endl;
//					return false;
//				}
//				else if (states_[j]->processChains_[i]->measurements_.empty()) {
//					std::cout << "Process measurement " << i << " for the state at t=" << states_[j]->t() << " contains no measurments!" << std::endl;
//					return false;
//				}
//
//				//Check for process measurement at or before each state
//				if (states_[j]->processChains_[i]->measurements_.front()->t() > states_[j]->t()) {
//					std::cout << "There is no process measurement at or before the state at t=" << states_[j]->t() <<
//							"; t_process_front=" << states_[j]->processChains_[i]->measurements_.front()->t() <<
//							"; t_state-t_meas=" << states_[j]->t()-states_[j]->processChains_[i]->measurements_.front()->t() << std::endl;
//					res = false;
//				}
//
//				//Check for process measurements coming at or after the following state
//				if (states_[j]->processChains_[i]->measurements_.back()->t() >= states_[j+1]->t()) {
//					std::cout << "There is a process measurement assigned to the state at t=" <<
//						states_[j]->t() << " which is at or after the following state at " << states_[j+1]->t() <<
//						"; t_process_back=" << states_[j]->processChains_[i]->measurements_.back()->t() << std::endl;
//					res = false;
//				}

				if (states_[j]->processChains_[i]->initialized()) {
					if (states_[j]->processChains_[i]->tStart() != states_[j]->t()) {
						std::cout << "ERROR: ProcessChain at t=" << states_[j]->processChains_[i]->tEnd() <<
								" incorrectly assigned to state at t=" << states_[j]->t() << std::endl;
						res = false;
					}

					states_[j]->processChains_[i]->check();
				}
			}
		}

		for (size_t i=0; i<numUpdateSensors_; ++i) {
			for (size_t j=0; j<states_.size(); ++j) {
				for (size_t k=0; k<states_[j]->updateMeasurements_[i].size(); ++k) {
					if (states_[j]->updateMeasurements_[i][k]->initialized()) {
						if (states_[j]->updateMeasurements_[i][k]->t() != states_[j]->t()) {
							std::cout << "Update measurement at " << states_[j]->updateMeasurements_[i][k]->t() << " incorrectly assigned to state at " << states_[j]->t() << std::endl;
							res = false;
						}
						states_[j]->updateMeasurements_[i][k]->check();
					}
				}
			}
		}

		return res;
	}

	void print() {
		for (size_t i=0; i<states_.size(); ++i) {
			std::cout << "State at T=" << states_[i]->t() << " has ";


			//Print process measurement times
			for (size_t j=0; j<numProcessSensors_; ++j) {
				if (!states_[i]->processChains_[j]->measurements_.empty()) {
					std::cout << states_[i]->processChains_[j]->measurements_.size() << " of process type " << j <<
							": t_front=" << states_[i]->processChains_[j]->measurements_.front()->t() <<
							": t_back=" << states_[i]->processChains_[j]->measurements_.back()->t() << std::endl;
//					for (int k=0; k<states_[i]->processChains_[j]->measurements_.size(); ++k) {
//						std::cout << states_[i]->processChains_[j]->measurements_[k]->t() << ", ";
//					}
				}
			}

			//Print update measurement times
			for (size_t j=0; j<numUpdateSensors_; ++j) {
				if (!states_[i]->updateMeasurements_[j].empty()) {
					std::cout << "Update type " << j << " at time: ";
					for (size_t k=0; k<states_[i]->updateMeasurements_[j].size(); ++k) {
						std::cout << states_[i]->updateMeasurements_[j][k]->t() << ", ";
					}
				}
				std::cout << "\n";
			}
			std::cout << std::endl;
		}
	}

	void deactivateParameters() {
		for (auto& state: states_)
			for (auto& param: state->parameters_)
				param.active_ = false;
	}

	size_t size() const { return states_.size(); }

	//todo Would be nicer if the user didn't need to pass a first state again, but then I need a method to clear the first state
	//todo This isn't needed now that the reset function is there?
//	void clear(std::shared_ptr<State> firstStatePtr) {
//		states_.clear();
//		states_.push_back(firstStatePtr);
//		firstStateInitialized = false;
//
//		for (int i=0; i<numProcessSensors_; ++i) processMeasStateIndex[i] = 0;
//		for (int i=0; i<numUpdateSensors_; ++i) updateMeasStateIndex[i] = 0;
//	}

	//This is used when tracking is stopped and restarted.
	void reset() {
		//Remove all states but the most recent one
		while (size() > 1)
			popState();

		//Reset the first state
		front()->reset();

		//todo This happens automatically, right?
//		for (int i=0; i<firstStatePtr->numProcessSensors_; ++i) processMeasStateIndex[i] = 0;
//		for (int i=0; i<firstStatePtr->numUpdateSensors_; ++i) updateMeasStateIndex[i] = 0;

		firstStateInitialized = false;
	}

	//This wipes out the whole state history. Used when the vector of stored states for a batch problem is reset to start another batch run.
	//Note that the measurement state index vectors are invalid after this is called.
	void clear() {
		states_.clear();
		firstStateInitialized = false;
	}

	//todo Are these necessary? Would be nice to keep states_ private
	std::shared_ptr<State>& operator[](size_t pos) { return states_[pos]; }
	const std::shared_ptr<const State> operator[](size_t pos) const { return std::const_pointer_cast<const State>(states_[pos]); }
	std::shared_ptr<State>& front() { return states_.front(); }
	std::shared_ptr<State>& back() { return states_.back(); }
	std::deque<std::shared_ptr<State>>::iterator begin() { return states_.begin(); }
	std::deque<std::shared_ptr<State>>::iterator end() { return states_.end(); }
	std::deque<std::shared_ptr<State>>::const_iterator begin() const { return states_.begin(); }
	std::deque<std::shared_ptr<State>>::const_iterator end() const { return states_.end(); }
	bool empty() const { return states_.empty(); }
//private:

	std::deque<std::shared_ptr<State>> states_;
	bool firstStateInitialized = false;

	const size_t numProcessSensors_;
	const size_t numUpdateSensors_;

	//These keep track of the last state to which measurements of different types were assigned
	std::vector<size_t> updateMeasStateIndex;
	std::vector<size_t> processMeasStateIndex;
};

} // namespace confusion


#endif /* INCLUDE_CONFUSION_STATEVECTOR_H_ */
