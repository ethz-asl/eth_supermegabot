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

#include "confusion/Diagram.h"

namespace confusion {

Diagram::Diagram(const ConFusor& in, std::string fname_) : confusor(in), fname(fname_) {
	getParameterPointerLocations();
	assignPrior();
	assignUpdateSensors();
	assignProcessSensors();
	drawDiagram();
}

void Diagram::getParameterPointerLocations() {
	const StateVector& stateVector = confusor.stateVector_;
	for (std::size_t stateIndex = 0; stateIndex < stateVector.size(); ++stateIndex) {
		for (std::size_t parameterIndex = 0; parameterIndex < stateVector[stateIndex]->parameters_.size(); ++parameterIndex) {
			const confusion::Parameter* parameterPointer = &(stateVector[stateIndex]->parameters_[parameterIndex]);
			Location location = {stateIndex, parameterIndex};
			parameterPointerLocations.insert(std::pair<const confusion::Parameter*, Location>(parameterPointer, location));
		}
	}
}

void Diagram::assignPrior() {
	const confusion::PriorConstraint& priorConstraint = confusor.priorConstraint_;
	processStateParameterVector(priorConstraint.statePriorParameters_, priorMeasurements.stateTargets);
	priorMeasurements.staticTargets = processStaticParameterVector(priorConstraint.staticPriorParameters_);

//	staticPriorTargets = processStaticParameterVector(priorConstraint.staticPriorParameters_);
//	priorTargets = processStateParameterVector(priorConstraint.statePriorParameters_);
}

void Diagram::assignUpdateSensors() {
	const StateVector& stateVector = confusor.stateVector_;
	for (std::size_t stateI = 0; stateI < stateVector.size(); ++stateI) {
		std::vector<MeasurementConnections> connectionsPerState;
		for (std::size_t sensorI = 0; sensorI < stateVector[stateI]->updateMeasurements_.size(); ++sensorI) {
			if (stateVector[stateI]->updateMeasurements_[sensorI].empty()) {
				continue;
			}
			//todo Need to loop over the update measurements of each type!!
			const std::shared_ptr<UpdateMeasurement> updateMeas = stateVector[stateI]->updateMeasurements_[sensorI][0];

			MeasurementConnections sensorConnections;
			processStateParameterVector(updateMeas->linkedStateParameters_, sensorConnections.stateTargets);
			sensorConnections.staticTargets = processStaticParameterVector(updateMeas->linkedStaticParameters_);
			sensorConnections.name = updateMeas->name();
			connectionsPerState.push_back(sensorConnections);
		}
		updateSensors.push_back(connectionsPerState);
	}
}

void Diagram::assignProcessSensors() {
	const StateVector& stateVector = confusor.stateVector_;
	for (std::size_t stateI = 0; stateI < stateVector.size(); ++stateI) {
		std::vector<MeasurementConnections> connectionsPerState;
		for (std::size_t sensorI = 0; sensorI < stateVector[stateI]->processChains_.size(); ++sensorI) {
			const std::shared_ptr<ProcessChain> processChain = stateVector[stateI]->processChains_[sensorI];

			MeasurementConnections sensorConnections;
			processStateParameterVector(processChain->linkedStartStateParameters_, sensorConnections.stateTargets);
			processStateParameterVector(processChain->linkedEndStateParameters_, sensorConnections.stateTargets);
			sensorConnections.staticTargets = processStaticParameterVector(processChain->linkedStaticParameters_);
			sensorConnections.name = processChain->name();
			connectionsPerState.push_back(sensorConnections);
		}
		processSensors.push_back(connectionsPerState);
	}
}

void Diagram::processStateParameterVector(const std::vector<confusion::Parameter*>& in, std::vector<Diagram::Location>& locations) {
	for (confusion::Parameter* parameterPointer : in) {
		if (parameterPointer == nullptr) {
			continue;
		}
		locations.push_back(parameterPointerLocations.find(parameterPointer)->second);
	}
}

std::vector<std::size_t> Diagram::processStaticParameterVector(const std::vector<confusion::Parameter*>& in) {
	std::vector<std::size_t> out;

	for (confusion::Parameter* staticParameterPointer : in) {
		int i = 0;
		for (std::map<double*, confusion::Parameter>::const_iterator it = confusor.staticParameters_.begin();
				it != confusor.staticParameters_.end(); ++it) {
			if (staticParameterPointer == &(it->second)) {
				out.push_back(i);
				continue;
			}
			++i;
		}
	}
	return out;
}



void Diagram::drawDiagram() {
	const int height = 1000;
	const int width = 1500;
	cv::Mat image(height, width, CV_8UC3, cv::Scalar(255, 255, 255));

	// Compute dimensions
	std::size_t maxParameterI = 0;
	for (auto&& parameter : parameterPointerLocations) {
		const std::size_t parameterI = parameter.second.parameterIndex;
		maxParameterI = std::max(maxParameterI, parameterI);
	}
	const std::size_t numParameters = maxParameterI + 1;

	dx = (width - 200) / (confusor.stateVector_.size());
	dy = (height - 100) / (3 + confusor.stateVector_.numProcessSensors_ + numParameters + confusor.stateVector_.numUpdateSensors_);


	dxStatic = (width - 200) / (confusor.staticParameters_.size() - 1);
	y0Static = dy;
	int y0Update = y0Static + 2 * dy;
	y0State = y0Update + (confusor.stateVector_.numUpdateSensors_) * dy;
	int y0Process = y0State + (numParameters) * dy;

	cv::Scalar processColor(0, 0, 255);
	drawConnections(image, processSensors, y0Process, processColor);

	cv::Scalar updateColor(0, 255, 0);
	drawConnections(image, updateSensors, y0Update, updateColor);

	const cv::Scalar priorColor(255, 255, 0);
	drawPrior(image, priorColor);

	drawStates(image, parameterPointerLocations);
	drawStaticParameters(image, confusor.staticParameters_);

	cv::putText(image, "Update sensors", cv::Point(width/2, height - 110),  cv::FONT_HERSHEY_SIMPLEX, 1, updateColor);
	cv::putText(image, "Process sensors", cv::Point(width/2, height - 60),  cv::FONT_HERSHEY_SIMPLEX, 1, processColor);
	cv::putText(image, "Prior", cv::Point(width/2, height - 10), cv::FONT_HERSHEY_SIMPLEX, 1, priorColor);

	cv::imwrite(fname, image);
	std::cout << "Wrote Confusor diagram to " << fname << std::endl;
}

void Diagram::drawConnections(cv::Mat& image, const std::vector<std::vector<Diagram::MeasurementConnections>>& targets,
		const int y0source, const cv::Scalar& color) const {
	for (std::size_t stateI = 0; stateI < targets.size(); ++stateI) {
		for (std::size_t sensorI = 0; sensorI < targets[stateI].size(); ++sensorI) {
			const int xSource = x0 + stateI * dx + dx/4;
			const int ySource = y0source + sensorI * dy;
			const cv::Point source(xSource, ySource);
			drawLinesToTargets(image, source, targets[stateI][sensorI].stateTargets, color);
			drawLinesToTargets(image, source, targets[stateI][sensorI].staticTargets, color);

			const int radius = 5;
			cv::circle(image, source, radius, color, -1);
			cv::putText(image, targets[stateI][sensorI].name, source + cv::Point(5, 3), cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
		}
	}
}

// to state parameters
void Diagram::drawLinesToTargets(cv::Mat& image, const cv::Point& source,
		const std::vector<Location>& targets, const cv::Scalar& color) const {
	for (auto&& target : targets) {
		const int xTarget = x0 + target.stateIndex * dx;
		const int yTarget = y0State + target.parameterIndex * dy;
		cv::line(image, source, cv::Point(xTarget, yTarget), color, 1);
	}
}

// to static targets
void Diagram::drawLinesToTargets(cv::Mat& image, const cv::Point& source,
		const std::vector<std::size_t>& targets, const cv::Scalar& color) const {
	for (auto&& target : targets) {
		const int xTarget = x0 + target * dxStatic;
		cv::line(image, source, cv::Point(xTarget, y0Static), color, 1);
	}
}

void Diagram::drawStates(cv::Mat& image, const std::map<const confusion::Parameter*, Location> parameterPointerLocations) const {

	cv::Scalar color(0, 0, 0);
	int radius = 5;
	for (auto&& parameter : parameterPointerLocations) {
		const std::size_t stateI = parameter.second.stateIndex;
		const std::size_t parameterI = parameter.second.parameterIndex;

		const int x = x0 + stateI * dx;
		const int y = y0State + parameterI * dy;

		cv::circle(image, cv::Point(x, y), radius, color, -1);

		cv::putText(image, parameter.first->name(), cv::Point(x + 5, y + 3), cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
	}

	cv::Point legendOrigin(100, 900);
	cv::circle(image, legendOrigin, radius, color, -1);
	cv::putText(image, "State parameters", legendOrigin + cv::Point(20, 10),  cv::FONT_HERSHEY_SIMPLEX, 1, color);
}

void Diagram::drawStaticParameters(cv::Mat& image, const StaticParameterVector& staticParameters) const {

	const cv::Scalar color(255, 0, 0);
	const int radius = 5;

	std::size_t i = 0;
	for (std::map<double*, confusion::Parameter>::const_iterator it = staticParameters.begin();
			it != staticParameters.end(); ++it) {
		const int x = x0 + i * dxStatic;
		cv::circle(image, cv::Point(x0 + i * dxStatic, y0Static), radius, color, -1);
		cv::putText(image, it->second.name(), cv::Point(x + 5, y0Static + 3), cv::FONT_HERSHEY_SIMPLEX, 0.5, color);

		++i;
	}

	cv::Point legendOrigin(100, 950);
	cv::circle(image, legendOrigin, radius, color, -1);
	cv::putText(image, "Static parameters", legendOrigin + cv::Point(20, 10),  cv::FONT_HERSHEY_SIMPLEX, 1, color);
}

void Diagram::drawPrior(cv::Mat& image, const cv::Scalar& color) const {
	const int radius = 5;

	for (auto&& stateTarget : priorMeasurements.stateTargets) {
		const int xSource = 25;
		const int ySource = y0State + stateTarget.parameterIndex * dy;
		const cv::Point source(xSource, ySource);

		const int xTarget = x0 + stateTarget.stateIndex * dx;
		const int yTarget = ySource;

		cv::line(image, source, cv::Point(xTarget, yTarget), color, 1);
		cv::circle(image, source, radius, color, -1);
	}

	for (auto&& staticTarget : priorMeasurements.staticTargets) {
		const int x = x0 + staticTarget * dxStatic;
		const int y = 25;
		const cv::Point source(x, y);

		cv::line(image, source, cv::Point(x, y0Static), color, 1);
		cv::circle(image, source, radius, color, -1);
	}
}

void Diagram::run() {

}


} //namespace confusion
