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

#ifndef INCLUDE_CON_FUSION_PRIORCONSTRAINT_H_
#define INCLUDE_CON_FUSION_PRIORCONSTRAINT_H_

#include <Eigen/Core>
#include "confusion/PriorCost.h"
#include "confusion/Parameter.h"

namespace confusion {

class PriorConstraint {
public:
	void addStateParameter(confusion::Parameter& param) {
//std::cout << "Adding state parameter of size " << param.size_ << " to the prior problem."
//			" priorConstraintActive_=" << param.priorConstraintActive_ << std::endl;
		if (param.priorConstraintActive()) {
			std::cout << "ERROR: Trying to add state parameter " << param.name() << ", which is already in the prior problem, to the prior problem!" << std::endl;
			return;
		}

		statePriorParameters_.push_back(&param);
		param.priorConstraintActive_ = true;

		statePortionLocalSize_ = statePortionLocalSize_ + param.localSize();
		priorParameterLocalSize_ = priorParameterLocalSize_ + param.localSize();
	}

	//It is only supported to call this before tracking has started
	//fixMarginalizedParams needs to be called after the parameters on the first state have been activated
	void addStateParameterAndActivate(confusion::Parameter& param) {
		if (param.priorConstraintActive()) {
            std::cout << "ERROR: Trying to add state parameter " << param.name() << ", which is already in the prior problem, to the prior problem!" << std::endl;
			return;
		}

		//We only support immediately activating prior state parameters before
		//tracking has started because marginalization doesnt support having state
		//parameters active that are newer than those within the prior problem.
		if (initialized_) {
			std::cout << "ERROR: PriorConstraint::addStateParameterAndActivate was "
                "called for parameter " << param.name() << " even though the PriorConstraint is already active!" << std::endl;
		}

#ifdef DEBUG_PRIOR
        std::cout << "Adding state parameter " << param.name() << " to the prior constraint. local size=" << param.localSize() << ", constant=" <<
          param.constant_ << std::endl;
#endif

		//Add the state parameter to the end of the state portion of the prior constraint
		Eigen::VectorXd priorErrorOffset(priorParameterLocalSize_ + param.localSize());
		priorErrorOffset.head(statePortionLocalSize_) = priorErrorOffset_.head(statePortionLocalSize_);
		priorErrorOffset.tail(staticPortionLocalSize_) = priorErrorOffset_.tail(staticPortionLocalSize_);
		priorErrorOffset.segment(statePortionLocalSize_, param.localSize()).setZero();
		priorErrorOffset_ = priorErrorOffset;

      Eigen::VectorXd bstar(priorParameterLocalSize_ + param.localSize());
      bstar.head(statePortionLocalSize_) = bstar_.head(statePortionLocalSize_);
      bstar.tail(staticPortionLocalSize_) = bstar_.tail(staticPortionLocalSize_);
      bstar.segment(statePortionLocalSize_, param.localSize()).setZero();
      bstar_ = bstar;

		Eigen::MatrixXd priorCostWeighting(priorParameterLocalSize_ + param.localSize(), priorParameterLocalSize_ + param.localSize());
		priorCostWeighting.topLeftCorner(statePortionLocalSize_, statePortionLocalSize_) = priorCostWeighting_.topLeftCorner(statePortionLocalSize_, statePortionLocalSize_);
		priorCostWeighting.bottomRightCorner(staticPortionLocalSize_, staticPortionLocalSize_) = priorCostWeighting_.bottomRightCorner(staticPortionLocalSize_, staticPortionLocalSize_);
		priorCostWeighting.topRightCorner(statePortionLocalSize_, staticPortionLocalSize_) = priorCostWeighting_.topRightCorner(statePortionLocalSize_, staticPortionLocalSize_);
		priorCostWeighting.bottomLeftCorner(staticPortionLocalSize_, statePortionLocalSize_) = priorCostWeighting_.bottomLeftCorner(staticPortionLocalSize_, statePortionLocalSize_);
		priorCostWeighting.block(0, statePortionLocalSize_, priorParameterLocalSize_ + param.localSize(), param.localSize()).setZero();
		priorCostWeighting.block(statePortionLocalSize_, 0, param.localSize(), priorParameterLocalSize_ + param.localSize()).setZero();
		priorCostWeighting.block(statePortionLocalSize_, statePortionLocalSize_, param.localSize(), param.localSize()) = param.initialConstraintWeighting_;
		priorCostWeighting_ = priorCostWeighting;

      Eigen::MatrixXd Hstar(priorParameterLocalSize_ + param.localSize(), priorParameterLocalSize_ + param.localSize());
      Hstar.topLeftCorner(statePortionLocalSize_, statePortionLocalSize_) = Hstar_.topLeftCorner(statePortionLocalSize_, statePortionLocalSize_);
      Hstar.bottomRightCorner(staticPortionLocalSize_, staticPortionLocalSize_) = Hstar_.bottomRightCorner(staticPortionLocalSize_, staticPortionLocalSize_);
      Hstar.topRightCorner(statePortionLocalSize_, staticPortionLocalSize_) = Hstar_.topRightCorner(statePortionLocalSize_, staticPortionLocalSize_);
      Hstar.bottomLeftCorner(staticPortionLocalSize_, statePortionLocalSize_) = Hstar_.bottomLeftCorner(staticPortionLocalSize_, statePortionLocalSize_);
      Hstar.block(0, statePortionLocalSize_, priorParameterLocalSize_ + param.localSize(), param.localSize()).setZero();
      Hstar.block(statePortionLocalSize_, 0, param.localSize(), priorParameterLocalSize_ + param.localSize()).setZero();
      Hstar.block(statePortionLocalSize_, statePortionLocalSize_, param.localSize(), param.localSize()) = param.initialConstraintWeighting_.transpose() * param.initialConstraintWeighting_;
      Hstar_ = Hstar;

		statePriorParameters_.push_back(&param);
		param.priorConstraintActive_ = true;

		statePortionLocalSize_ = statePortionLocalSize_ + param.localSize();
		priorParameterLocalSize_ = priorParameterLocalSize_ + param.localSize();
	}

	void clearStateParameters() {
		for (auto& param: statePriorParameters_)
			param->priorConstraintActive_ = false;
		statePriorParameters_.clear();
		statePortionLocalSize_ = 0;
		priorParameterLocalSize_ = staticPortionLocalSize_;
	}

	void addStaticParameter(confusion::Parameter& param) {
		if (param.constant_ || param.priorConstraintActive()) {
			return;
		}
#ifdef DEBUG_PRIOR
		std::cout << "Adding static parameter " << param.name() << " to the prior constraint. local size=" << param.localSize() << ", constant=" <<
		param.constant_ << ", prior active=" << param.priorConstraintActive() <<
		", immediatelyAddToPrior=" << param.immediatelyAddToPrior() << std::endl;
#endif
		staticPriorParameters_.push_back(&param);
		param.priorConstraintActive_ = true;
		param.priorConstraintLocalPosition_ = staticPortionLocalSize_;

		if (param.immediatelyAddToPrior()) {
			//Add the parameter to the end of the prior constraint parameter vector
			Eigen::VectorXd priorErrorOffset(priorParameterLocalSize_ + param.localSize());
			priorErrorOffset.head(priorParameterLocalSize_) = priorErrorOffset_;
			priorErrorOffset.tail(param.localSize()).setZero();
			priorErrorOffset_ = priorErrorOffset;

          Eigen::VectorXd bstar(priorParameterLocalSize_ + param.localSize());
          bstar.head(priorParameterLocalSize_) = bstar_;
          bstar.tail(param.localSize()).setZero();
          bstar_ = bstar;

			Eigen::MatrixXd priorCostWeighting(priorParameterLocalSize_ + param.localSize(), priorParameterLocalSize_ + param.localSize());
			priorCostWeighting.topLeftCorner(priorParameterLocalSize_, priorParameterLocalSize_) = priorCostWeighting_.topLeftCorner(priorParameterLocalSize_, priorParameterLocalSize_);
			priorCostWeighting.block(0, priorParameterLocalSize_, priorParameterLocalSize_ + param.localSize(), param.localSize()).setZero();
			priorCostWeighting.block(priorParameterLocalSize_, 0, param.localSize(), priorParameterLocalSize_ + param.localSize()).setZero();
			priorCostWeighting.block(priorParameterLocalSize_, priorParameterLocalSize_, param.localSize(), param.localSize()) = param.initialConstraintWeighting_;
			priorCostWeighting_ = priorCostWeighting;
//std::cout << "static: priorCostWeighting size: " << priorCostWeighting_.rows() << std::endl;
//std::cout << "static: priorCostWeighting:\n" << priorCostWeighting_ << std::endl;

          Eigen::MatrixXd Hstar(priorParameterLocalSize_ + param.localSize(), priorParameterLocalSize_ + param.localSize());
          Hstar.topLeftCorner(priorParameterLocalSize_, priorParameterLocalSize_) = Hstar_.topLeftCorner(priorParameterLocalSize_, priorParameterLocalSize_);
          Hstar.block(0, priorParameterLocalSize_, priorParameterLocalSize_ + param.localSize(), param.localSize()).setZero();
          Hstar.block(priorParameterLocalSize_, 0, param.localSize(), priorParameterLocalSize_ + param.localSize()).setZero();
          Hstar.block(priorParameterLocalSize_, priorParameterLocalSize_, param.localSize(), param.localSize()) = param.initialConstraintWeighting_.transpose() * param.initialConstraintWeighting_;
          Hstar_ = Hstar;

			//Fix the value of the parameter immediately as well
			//Note that this step is not there for adding state parameters
			//because we only support that the initially constrained state parameters
			//are always added first
			marginalizedParams_.push_back(confusion::FixedParameter(param));
			if (param.isRandomWalkProcessActive())
				priorParams_.push_back(param.rwpPriorSideParameterAddress());
			else
				priorParams_.push_back(param.data_);
//std::cout << "Added parameter " << param.name() << " to prior constraint and immediately activated it. W=\n" << param.initialConstraintWeighting_ << std::endl;
		}

		staticPortionLocalSize_ = staticPortionLocalSize_ + param.localSize();
		priorParameterLocalSize_ = priorParameterLocalSize_ + param.localSize();
	}

	//Note that this will invalidate the marginalizedParams and the prior problem
	//so these need to be re-constructed.
	//todo Hold a flag indicating if these things are valid to notify the user if they are doing things in the wrong order?
	void removeStaticParameter(confusion::Parameter& param) {
		if (!param.priorConstraintActive()) {
			std::cout << "ERROR: Trying to remove a static parameter from the prior problem that isn't in the prior problem!?" << std::endl;
			return;
		}

		//todo Store the parameters by data address in a map to speed this up
		int index = -1;
		int i = 0;
		for (auto& p: staticPriorParameters_) {
			if (&param == p) {
				index = i;
			}
			else if (index >= 0) {
				//We need to shift the position of the parameters following the one removed
				p->priorConstraintLocalPosition_ -= param.localSize();
			}
			++i;
		}

		if (index < 0) {
			std::cout << "ERROR: Didn't find the static parameter to remove in the prior constraint!?" << std::endl;
			return;
		}

		staticPriorParameters_.erase(staticPriorParameters_.begin()+index);
		param.priorConstraintActive_ = false;

		staticPortionLocalSize_ -= param.localSize();
		priorParameterLocalSize_ -= param.localSize();

#ifdef DEBUG_PRIOR
      std::cout << "Reomved static parameter " << param.name() << " from the prior constraint" << std::endl;
#endif
	}

	//Fix the value of the parameters in the prior cost for the next problem
	//Build the vector of prior parameter blocks for problem computation
	void fixMarginalizedParams() {
		marginalizedParams_.clear();
		priorParams_.clear();
		for (const auto& p: statePriorParameters_) {
			marginalizedParams_.emplace_back(confusion::FixedParameter(*p));
			if (p->isRandomWalkProcessActive())
				priorParams_.push_back(p->rwpPriorSideParameterAddress());
			else
				priorParams_.push_back(p->data_);
		}
		for (const auto& p: staticPriorParameters_) {
			marginalizedParams_.emplace_back(confusion::FixedParameter(*p));
			if (p->isRandomWalkProcessActive())
				priorParams_.push_back(p->rwpPriorSideParameterAddress());
			else
				priorParams_.push_back(p->data_);
		}

#ifdef DEBUG_PRIOR
		std::cout << "Fixed " << statePriorParameters_.size() << " state parameters and " <<
				staticPriorParameters_.size() << " static parameters for marginalization" << std::endl;
#endif
	}

	bool check() {
		if (statePortionLocalSize_ + staticPortionLocalSize_ != priorParameterLocalSize_) {
			std::cout << "ERROR: PriorConstraint sizes do not matchup!" << std::endl;
			return false;
		}

		int statePortionLocalSize = 0;
		for (const auto& p: statePriorParameters_)
			statePortionLocalSize += p->localSize();
		if (statePortionLocalSize != statePortionLocalSize_) {
			std::cout << "ERROR: PriorConstraint statePriorParameter size (" << statePortionLocalSize <<
					") does not match statePortionLocalSize (" << statePortionLocalSize_ << ")!" << std::endl;
			return false;
		}

		int staticPortionLocalSize = 0;
		for (const auto& p: staticPriorParameters_) {
			if (p->priorConstraintLocalPosition_ != staticPortionLocalSize)
				std::cout << "ERROR in PriorConstraint. Mismatch in a staticParameter's local position!" << std::endl;
			staticPortionLocalSize += p->localSize();
		}
		if (staticPortionLocalSize != staticPortionLocalSize_) {
			std::cout << "ERROR: PriorConstraint staticPriorParameter size (" << staticPortionLocalSize <<
					") does not match staticPortionLocalSize (" << staticPortionLocalSize_ << ")!" << std::endl;
			return false;
		}

		if (priorErrorOffset_.size() != priorParameterLocalSize_) {
			std::cout << "ERROR: PriorConstraint priorErrorOffset has incorrect size! size=" << priorErrorOffset_.size() <<
					", expected=" << priorParameterLocalSize_ << std::endl;
			std::cout << "Static portion size: " << staticPortionLocalSize_ << ", state portion size: " << statePortionLocalSize_ << std::endl;
			return false;
		}

		if (priorCostWeighting_.rows() != priorParameterLocalSize_ || priorCostWeighting_.cols() != priorParameterLocalSize_) {
			std::cout << "ERROR: PriorConstraint priorCostWeighting has incorrect size!" << std::endl;
			return false;
		}

      if (priorErrorOffset_.size() != bstar_.size()) {
        std::cout << "ERROR: PriorConstraint priorErrorOffset and bstar sizes don't match! priorErrorOffset size=" << priorErrorOffset_.size() <<
                  ", bstar size=" << bstar_.size() << std::endl;
        return false;
      }

      if (priorCostWeighting_.rows() != Hstar_.rows() || priorCostWeighting_.cols() != Hstar_.cols()) {
        std::cout << "ERROR: PriorConstraint priorCostWeighting and Hstar sizes don't match! priorCostWeighting size=" << priorCostWeighting_.rows() << "," << priorCostWeighting_.cols() <<
                  ", Hstar size=" << Hstar_.rows() << "," << Hstar_.cols() << std::endl;
        return false;
      }

		return true;
	}

	void addPriorCostToProblem(ceres::Problem* problem, double* dtFirstState = nullptr) {
      priorCostPtr = std::make_shared<confusion::PriorCost>(marginalizedParams_, priorCostWeighting_, priorErrorOffset_);
      problem->AddResidualBlock(priorCostPtr.get(), NULL, priorParams_);

      //Activate the parameters linked to the prior constraint
      //Also process the random walk processes for the linked static parameters
      for (auto& p: statePriorParameters_)
          p->active_ = true;
      for (auto& p: staticPriorParameters_) {
        p->active_ = true;

        if (p->isRandomWalkProcessAttached()) {
          if (dtFirstState) {
            //Update the RWP. This will also activate it if it hasn't yet been activated.
            p->updateRandomWalkProcess(*dtFirstState);
          }

          if (p->isRandomWalkProcessActive()) {
            p->addRandomWalkProcessToProblem(problem);
          }
        }
      }
	}

	void print() {
	  std::cout << "PriorConstraint has " << priorParams_.size() << " parameters linked to it of local size -- state=" << statePortionLocalSize_ <<
	      ", static=" << staticPortionLocalSize_ << std::endl;
	  for (auto &param: statePriorParameters_)
	    std::cout << "Prior constrained state parameter " << param->name() << " of local size " << param->localSize() << std::endl;
      for (auto &param: staticPriorParameters_)
        std::cout << "Prior constrained static parameter " << param->name() << " of local size " << param->localSize() << std::endl;
	}

	void initialize() {
		initialized_ = true;
	}

	bool initialized() {
		return initialized_;
	}

	void reset() {
		initialized_ = false;
	}

	//This also removes the static parameter portion to remove all prior information
	void completeReset() {
		//Clear the state parameters
		clearStateParameters();
		reset();

		//Also clear the static parameters
		for (auto& param: staticPriorParameters_)
			param->priorConstraintActive_ = false;
		staticPriorParameters_.clear();
		staticPortionLocalSize_ = 0;
		priorParameterLocalSize_ = statePortionLocalSize_;

		//Wipe out everything else
		priorErrorOffset_.resize(0);
		priorCostWeighting_.resize(0,0);
        bstar_.resize(0);
        Hstar_.resize(0,0);
		marginalizedParams_.clear();
		priorCostPtr = nullptr;
		priorParams_.clear();
	}

	//Members governing the prior cost. These are set during state marginalization and manipulated when tracking is stopped and static parameters are added/removed from the prior constraint.
	Eigen::VectorXd priorErrorOffset_; ///< (J_p^T)^dagger bstar
	Eigen::MatrixXd priorCostWeighting_; ///< J_p
    Eigen::MatrixXd Hstar_; ///< Hstar = J_p^T J_p
    Eigen::VectorXd bstar_;

	std::vector<confusion::Parameter*> statePriorParameters_;
	std::vector<confusion::Parameter*> staticPriorParameters_;

	int statePortionLocalSize_ = 0;
	int staticPortionLocalSize_ = 0;
	int priorParameterLocalSize_ = 0;

	std::vector<confusion::FixedParameter> marginalizedParams_;

	std::shared_ptr<confusion::PriorCost> priorCostPtr;
	std::vector<double*> priorParams_; //This is sent at problem.AddResidualBlock to tell Ceres which parameters are involved

private:
	bool initialized_ = false;
};

}

#endif /* INCLUDE_CON_FUSION_PRIORCONSTRAINT_H_ */
