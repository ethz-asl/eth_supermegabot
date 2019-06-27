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

#ifndef INCLUDE_CONFUSION_STATICPARAMETERVECTOR_H_
#define INCLUDE_CONFUSION_STATICPARAMETERVECTOR_H_

#include <map>
#include <list>
#include "confusion/Parameter.h"

namespace confusion {

class StaticParameterVector {
public:
	StaticParameterVector() { }

	//Copy constructor (used in BatchFusor)
	StaticParameterVector(const StaticParameterVector& sp):
		parameterMap_(sp.parameterMap_),
		globalSize_(sp.globalSize_) { }

	StaticParameterVector& operator=(const StaticParameterVector& spv) {
		if(this == &spv)
			return *this;

		parameterMap_ = spv.parameterMap_;
		globalSize_ = spv.globalSize_;

		return *this;
	}

	//	size_t localSize() const { return priorLocalSize_; }
	size_t numParameters() const { return parameterMap_.size(); }

	void addParameter(confusion::Parameter param) {
		parameterMap_.emplace(std::make_pair(param.data_, param));
//		parameterMap_[param.data_] = param;
		globalSize_ += param.size();
	}

	//todo Returned address should be constant
	confusion::Parameter* getParameter(double* dataPtr) {
		confusion::Parameter* parameter;
		try {
			parameter = &(parameterMap_.at(dataPtr));
		}
		catch (const std::out_of_range& oor) {
			std::cerr << "ERROR: Didn't find parameter at " << dataPtr << " in the StaticParameterVector!" << std::endl;
			return nullptr;
		}
		return parameter;
	}

//	//Return -1 if the parameter isn't found
//	int getParameterIndex(double* dataPtr) {
//		int parameterIndex = -1;
//		try {
//			parameterIndex = parameterMap_.at(dataPtr);
//		}
//		catch (const std::out_of_range& oor) {
//			std::cerr << "ERROR: Didn't find parameter at " << dataPtr << " in the StaticParameterVector!" << std::endl;
//		}
//		return parameterIndex;
//	}

	void deactivateParameters() {
		for (auto& p: parameterMap_)
			p.second.active_ = false;
	}

	void removeParameter(double* dataPtr) {
		int numParamsRemoved = parameterMap_.erase(dataPtr);
		if (numParamsRemoved != 1) {
			std::cout << "Removng parameter from the StaticParameterVector actually removed " << numParamsRemoved << " parameters. Something is probably wrong!" << std::endl;
		}
	}

    void attachStaticParameterRandomWalkProcess(double* data, const double &processNoise) {
	  confusion::Parameter* parameter = getParameter(data);
	  if (!parameter) {
	  	std::cout << "ERROR: Could not find desired parameter for StaticParameterVector::setStaticParameterRandomWalkProcess." << std::endl;
	  	return;
	  }
      parameter->attachRandomWalkProcess(processNoise);
	}

	void detachStaticParameterRandomWalkProcess(double* data) {
      confusion::Parameter* parameter = getParameter(data);
      if (!parameter) {
        std::cout << "ERROR: Could not find desired parameter for StaticParameterVector::deactivateStaticParameterRandomWalkProcess." << std::endl;
        return;
      }

      parameter->detachRandomWalkProcess();
	}

    void print() {
      for (auto& p: parameterMap_) {
        std::cout << "Static parameter " << p.second.name() << ": size=" << p.second.size() <<
                  "; constant=" << p.second.isConstant() <<
                  "; priorConstr=" << p.second.priorConstraintActive() << std::endl;
      }
    }

	//todo Add tests?
	bool check() {
		return true;
	}

	const size_t globalSize() const { return globalSize_; }

	//Don't expose standard methods to remove or modify elements
//	confusion::Parameter& operator[](size_t pos) { return parameters_[pos]; }
//	const confusion::Parameter& operator[](size_t pos) const { return parameters_[pos]; }
//	confusion::Parameter& front() { return parameterMap_.front(); }
//	confusion::Parameter& back() { return parameters_.back(); }
	std::map<double*, confusion::Parameter>::iterator begin() { return parameterMap_.begin(); }
	std::map<double*, confusion::Parameter>::iterator end() { return parameterMap_.end(); }
	std::map<double*, confusion::Parameter>::const_iterator begin() const { return parameterMap_.begin(); }
	std::map<double*, confusion::Parameter>::const_iterator end() const { return parameterMap_.end(); }
	std::size_t size() const { return parameterMap_.size(); }

private:
	std::map<double*, confusion::Parameter> parameterMap_;
	size_t globalSize_ = 0;
};

} // namespace confusion

#endif /* INCLUDE_CONFUSION_STATICPARAMETERVECTOR_H_ */
