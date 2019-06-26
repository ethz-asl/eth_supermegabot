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

#ifndef INCLUDE_CONFUSION_PROCESSMEASUREMENT_H_
#define INCLUDE_CONFUSION_PROCESSMEASUREMENT_H_

//#include <memory>
//#include <deque>
//#include <string>
//#include "confusion/Parameter.h"
//#include "confusion/StaticParameterVector.h"

namespace confusion {

/**
 * Base class for process measurements. This is typically just a bin for the
 * measurement data and references to any relevant calibration data, with the
 * sensor model and cost function implemented in the corresponding ProcessChain.
 */
class ProcessMeasurement {
public:
	/**
	 * Construct a process measurement
	 *
	 * @param measType The integer identifier for the measurement type.
	 * @param t Measurement timestamp
	 */
	ProcessMeasurement(int measType, double t): measType_(measType), t_(t) { }
	virtual ~ProcessMeasurement() { }

	const int measType() const { return measType_; }
	const double t() const { return t_; }

protected:
	int measType_; ///< Measurement index identifier
	double t_; ///< Timestamp
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_PROCESSMEASUREMENT_H_ */
