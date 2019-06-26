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

#ifndef INCLUDE_CONFUSION_DIAGRAM_H_
#define INCLUDE_CONFUSION_DIAGRAM_H_

#include <map>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "confusion/ConFusor.h"
#include "confusion/StateVector.h"
#include "confusion/StaticParameterVector.h"

namespace confusion {

//Note that the diagram should be created AFTER solving the MHE problem so that the links are established
class Diagram {
public:
	Diagram(const ConFusor& in, std::string  = "ConFusorDiagram.png");
private:
	struct Location {
		std::size_t stateIndex;
		std::size_t parameterIndex;
	};

	struct MeasurementConnections {
		std::vector<Location> stateTargets;
		std::vector<std::size_t> staticTargets;
		std::string name;
	};

	// Detection logic
	const ConFusor& confusor;
	std::map<const confusion::Parameter*, Location> parameterPointerLocations;

	MeasurementConnections priorMeasurements;
	std::vector<std::vector<MeasurementConnections>> processSensors;
	std::vector<std::vector<MeasurementConnections>> updateSensors;

	void getParameterPointerLocations();
	void assignPrior();
	void assignUpdateSensors();
	void assignProcessSensors();
	void processStateParameterVector(const std::vector<confusion::Parameter*>& in, std::vector<Diagram::Location>& locations);
	std::vector<std::size_t> processStaticParameterVector(const std::vector<confusion::Parameter*>& in);
	void run();

	// Diagram
	// Settings
	const int x0 = 100;
	int dx;
	int dxStatic;
	int y0State;
	int y0Static;
	int dy;

	void drawDiagram();

	void drawConnections(cv::Mat& image, const std::vector<std::vector<Diagram::MeasurementConnections>>& targets,
			const int y0source, const cv::Scalar& color) const;
	void drawLinesToTargets(cv::Mat& image,
			const cv::Point& source, const std::vector<Location>& targets, const cv::Scalar& color) const;
	void drawLinesToTargets(cv::Mat& image, const cv::Point& source,
			const std::vector<std::size_t>& targets, const cv::Scalar& color) const;
	void drawStates(cv::Mat& image, const std::map<const confusion::Parameter*, Location> parameterPointerLocations) const;
	void drawStaticParameters(cv::Mat& image, const StaticParameterVector& staticParameters) const;
	void drawPrior(cv::Mat& image, const cv::Scalar& color) const;

	std::string fname;
};

} //namespace confusion


#endif /* INCLUDE_CONFUSION_DIAGRAM_H_ */
