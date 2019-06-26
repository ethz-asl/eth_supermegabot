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

#include "confusion/ConFusor.h"
#include "confusion/Diagram.h"

class TestUpdateMeas : public UpdateMeasBase {
public:
	TestUpdateMeas(int measType, double t) : UpdateMeasBase(measType, t) { }

	ceres::ResidualBlockId addCostToProblem(ceres::Problem* problem) {
		return ceres::ResidualBlockId();
	}

	int residualDimension() {
		return 1;
	}
};

class TestProcessChain : public ProcessChain {
	bool addCostToProblem(ceres::Problem* problem, ceres::ResidualBlockId& residualId) {
		return true;
	}

	int residualDimension() {
		return 1;
	}
};

class TestState : public State {
public:
	TestState(double t, int numProcessSensors, int numUpdateSensors) : State(t, numProcessSensors, numUpdateSensors) {
		for (int i = 0; i < numProcessSensors; ++i) {
			processChains_[i] = std::make_shared<TestProcessChain>();
		}
		parameters_.push_back(confusion::Parameter(state1.data(), 3));
		parameters_.push_back(confusion::Parameter(state2.data(), 3));
		check();
	}

	TestState(const TestState* in) : State(in->t_, in->numProcessSensors_, in->numUpdateSensors_){
		for (int i = 0; i < in->numProcessSensors_; ++i) {
			processChains_[i] = std::make_shared<TestProcessChain>();
		}
		parameters_.push_back(in->parameters_[0].clone(state1.data()));
		parameters_.push_back(in->parameters_[1].clone(state2.data()));
		check();
	}

	bool initFirstState(const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>>& processMeasBuffer,
			const std::vector<std::deque<std::shared_ptr<UpdateMeasBase>>>& updateMeasBuffer,
			StaticParameterVector& staticParameters) {
		return true;
	}

	std::shared_ptr<State> sharedBasePointer() {
		return std::dynamic_pointer_cast<State>(std::make_shared<TestState>(this));
	}

	std::shared_ptr<State> createNextState(
			const std::vector<std::deque<std::shared_ptr<ProcessMeasurement>>>& processMeasBuffer,
			const std::vector<std::deque<std::shared_ptr<UpdateMeasBase>>>& updateMeasBuffer,
			StaticParameterVector& staticParameters) {
		TestState out(this);

		if (updateMeasBuffer[0].empty()) {
			std::cout << "Next state not created\n";
			return nullptr;
		}

		double tMin = out.t_ + 1;
		for (auto&& updateMeas : updateMeasBuffer) {
			if (!updateMeas.empty()) {
				tMin = std::min(updateMeas.front()->t_, tMin);
			}
		}
		out.t_ = tMin;
		return out.sharedBasePointer();
	}

private:
	Eigen::Vector3d state1;
	Eigen::Vector3d state2;
};

// Creates clean setup of Confusor, with one update measurement at t=1, and at t=2, and process measurements from 0:0.25:2.75
void setupConfusor(ConFusor& confusor, const int numProcessSensors, const int numUpdateSensors) {
	int freq = 4;
	for (int i = 0; i <= 2*freq; i += 1) {
		double t = i / static_cast<double>(freq);
		for (int sensor = 0; sensor < numProcessSensors; ++sensor) {
			confusor.addProcessMeasurement(std::make_shared<ProcessMeasurement>(ProcessMeasurement(sensor, t)));
		}
		if (i > 0 && i % freq == 0) {
			for (int sensor = 0; sensor < numUpdateSensors; ++sensor) {
				confusor.addUpdateMeasurement(std::make_shared<TestUpdateMeas>(TestUpdateMeas(sensor, t)));
			}
			confusor.assignMeasurements();
		}
	}
}




int main() {
	const int numProcessSensors = 3;
	const int numUpdateSensors = 2;
	double tState = 0;
	TestState firstState(tState, numProcessSensors,  numUpdateSensors);
	ConFusor confusor(firstState.sharedBasePointer());
	setupConfusor(confusor, numProcessSensors, numUpdateSensors);


	std::cout << "-------------------------\n\n\n\n";

	Diagram diagram(confusor);

	confusor.stateVector_.print();
}
